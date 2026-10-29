//! The viewer layout a recording opens with (its blueprint).
//!
//! Three views, one above the other; everything else is a click away in the
//! blueprint panel, which starts open:
//!
//! | view | what |
//! |---|---|
//! | `Ego` (3D) | the scene from behind the ego, following it (origin `world/ego/pose`) |
//! | `Conditions` (state timeline) | `scenario/conditions/**`: when each pass/fail condition held |
//! | `Actions` (state timeline) | `scenario/actions/**`: when each action started, ran and ended; with plans, `scenario/turn_indicator` too |
//!
//! A run with a driver policy's plans (`plan` records) gets a second column:
//!
//! | view | what |
//! |---|---|
//! | `Plan speed` (2D) | `plan_profile/speed`: the current plan's speed against its time ahead, and the previous plan's |
//! | `Plan` (time series) | `plan/**`: the policy's inference time and diagnostics |
//!
//! The blueprint is written with the blueprint archetypes directly: the Rust
//! SDK's `Spatial3DView` cannot set the eye.

use rerun::external::re_log_types::BlueprintActivationCommand;
use rerun::external::re_sdk_types::blueprint::archetypes::{
    ContainerBlueprint, EyeControls3D, PanelBlueprint, TimePanelBlueprint, ViewBlueprint,
    ViewContents, ViewportBlueprint,
};
use rerun::external::re_sdk_types::blueprint::components::{
    ColumnShare, ContainerKind, Eye3DKind, Fps, IncludedContent, PanelState, QueryExpression,
    RootContainer, RowShare, TimelineName, ViewClass,
};
use rerun::{RecordingStream, RecordingStreamBuilder};

/// Where the ego's pose is (a `Transform3D`, its yaw only): the chase view's origin.
pub const EGO_POSE: &str = "world/ego/pose";
/// The chase eye, in the ego's frame (x forward, z up), metres.
const EYE_POSITION: [f32; 3] = [-14.0, 0.0, 6.0];
const EYE_TARGET: [f32; 3] = [12.0, 0.0, 1.0];

type BpResult<T> = Result<T, String>;

fn err(e: impl std::fmt::Display) -> String {
    format!("rerun: {e}")
}

/// A blueprint id: fixed, so that the same log gives the same file.
fn id(n: u8) -> ([u8; 16], String) {
    let mut bytes = [0u8; 16];
    bytes[6] = 0x40; // version 4
    bytes[8] = 0x80; // RFC 4122 variant
    bytes[15] = n;
    let hex: String = bytes.iter().map(|b| format!("{b:02x}")).collect();
    let text = format!(
        "{}-{}-{}-{}-{}",
        &hex[0..8],
        &hex[8..12],
        &hex[12..16],
        &hex[16..20],
        &hex[20..32]
    );
    (bytes, text)
}

struct Builder {
    bp: RecordingStream,
    next: u8,
}

impl Builder {
    fn view(
        &mut self,
        class: &str,
        name: &str,
        origin: &str,
        contents: &[&str],
    ) -> BpResult<String> {
        self.next += 1;
        let path = format!("view/{}", id(self.next).1);
        self.bp
            .log(
                format!("{path}/ViewContents"),
                &ViewContents::new(contents.iter().map(|q| QueryExpression((*q).into()))),
            )
            .map_err(err)?;
        self.bp
            .log(
                path.as_str(),
                &ViewBlueprint::new(ViewClass(class.into()))
                    .with_display_name(name)
                    .with_space_origin(origin),
            )
            .map_err(err)?;
        Ok(path)
    }

    /// A container of `kind` holding `contents`, sized by `shares`; its id
    /// and its path, by which another container includes it.
    fn container(
        &mut self,
        kind: ContainerKind,
        contents: &[String],
        shares: &[f32],
    ) -> BpResult<([u8; 16], String)> {
        self.next += 1;
        let (bytes, text) = id(self.next);
        let path = format!("container/{text}");
        let mut container = ContainerBlueprint::new(kind)
            .with_contents(contents.iter().map(|c| IncludedContent(c.as_str().into())));
        container = if kind == ContainerKind::Horizontal {
            container.with_col_shares(shares.iter().map(|s| ColumnShare((*s).into())))
        } else {
            container.with_row_shares(shares.iter().map(|s| RowShare((*s).into())))
        };
        self.bp.log(path.as_str(), &container).map_err(err)?;
        Ok((bytes, path))
    }
}

/// Sends the layout into `rec`, active at once. `chase`: there is an ego to
/// follow (`EGO_POSE` is logged); `plans`: a driver policy's plans are.
pub fn send(
    rec: &RecordingStream,
    application_id: &'static str,
    recording_id: &str,
    chase: bool,
    plans: bool,
) -> BpResult<()> {
    let (bp, storage) = RecordingStreamBuilder::new(application_id)
        .recording_id(format!("{recording_id}.blueprint"))
        .blueprint()
        .memory()
        .map_err(err)?;
    // The viewer reads blueprint data on this timeline.
    bp.set_time_sequence("blueprint", 0);
    let mut b = Builder { bp, next: 0 };

    let scene = if chase {
        let path = b.view(
            "3D",
            "Ego",
            &format!("/{EGO_POSE}"),
            &["/world/**", "/map/**"],
        )?;
        b.bp.log(
            format!("{path}/EyeControls3D"),
            &EyeControls3D::new()
                .with_kind(Eye3DKind::Orbital)
                .with_position(EYE_POSITION)
                .with_look_target(EYE_TARGET)
                .with_eye_up([0.0, 0.0, 1.0]),
        )
        .map_err(err)?;
        path
    } else {
        b.view("3D", "World", "/", &["/world/**", "/map/**"])?
    };
    let conditions = b.view(
        "StateTimeline",
        "Conditions",
        "/scenario/conditions",
        &["$origin/**"],
    )?;
    let action_contents: &[&str] = if plans {
        &["$origin/**", "/scenario/turn_indicator"]
    } else {
        &["$origin/**"]
    };
    let actions = b.view(
        "StateTimeline",
        "Actions",
        "/scenario/actions",
        action_contents,
    )?;
    let (column, column_path) = b.container(
        ContainerKind::Vertical,
        &[scene, conditions, actions],
        &[3.0, 1.0, 1.0],
    )?;
    let root = if plans {
        let speed = b.view("2D", "Plan speed", "/plan_profile", &["$origin/**"])?;
        let series = b.view("TimeSeries", "Plan", "/plan", &["$origin/**"])?;
        let (_, side) = b.container(ContainerKind::Vertical, &[speed, series], &[1.0, 1.0])?;
        b.container(ContainerKind::Horizontal, &[column_path, side], &[2.0, 1.0])?
            .0
    } else {
        column
    };

    let bp = &b.bp;
    bp.log(
        "viewport",
        &ViewportBlueprint::new()
            .with_root_container(RootContainer(root.into()))
            .with_auto_layout(false)
            .with_auto_views(false),
    )
    .map_err(err)?;
    // The rest is in the blueprint panel, open, to be added from there.
    bp.log(
        "blueprint_panel",
        &PanelBlueprint::new().with_state(PanelState::Expanded),
    )
    .map_err(err)?;
    bp.log(
        "selection_panel",
        &PanelBlueprint::new().with_state(PanelState::Collapsed),
    )
    .map_err(err)?;
    bp.log(
        "time_panel",
        &TimePanelBlueprint::new()
            .with_state(PanelState::Collapsed)
            .with_timeline(TimelineName("sim_time".into()))
            .with_fps(Fps(20.0.into())),
    )
    .map_err(err)?;

    let msgs = storage.take();
    let blueprint_id = msgs
        .first()
        .map(|m| m.store_id().clone())
        .ok_or_else(|| err("empty blueprint"))?;
    rec.send_blueprint(
        msgs,
        BlueprintActivationCommand {
            blueprint_id,
            make_active: true,
            make_default: true,
        },
    );
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::id;

    #[test]
    fn ids_are_uuids() {
        let (bytes, text) = id(3);
        assert_eq!(text, "00000000-0000-4000-8000-000000000003");
        assert_eq!(bytes[15], 3);
    }
}
