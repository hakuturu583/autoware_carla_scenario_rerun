//! The viewer layout a recording opens with (its blueprint).
//!
//! Few views, each with one job:
//!
//! | view | what |
//! |---|---|
//! | `Ego` (3D) | the scene from behind the ego, following it (origin `world/ego/pose`) |
//! | `Scenario` (state timeline) | `scenario/**`: the run, its conditions, its lights |
//! | `Speed`, `Control`, `Result`, `Log` (tabs) | the ego's speed and control, the verdict, the events |
//!
//! The blueprint is written with the blueprint archetypes directly: the Rust
//! SDK's `Spatial3DView` cannot set the eye.

use rerun::external::re_log_types::BlueprintActivationCommand;
use rerun::external::re_sdk_types::blueprint::archetypes::{
    ContainerBlueprint, EyeControls3D, PanelBlueprint, TimePanelBlueprint, ViewBlueprint,
    ViewContents, ViewportBlueprint,
};
use rerun::external::re_sdk_types::blueprint::components::{
    ActiveTab, ColumnShare, ContainerKind, Eye3DKind, Fps, IncludedContent, PanelState,
    QueryExpression, RootContainer, RowShare, TimelineName, ViewClass,
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

    fn container(
        &mut self,
        kind: ContainerKind,
        contents: &[String],
        shares: &[f32],
        active: Option<&str>,
    ) -> BpResult<(String, [u8; 16])> {
        self.next += 1;
        let (bytes, text) = id(self.next);
        let path = format!("container/{text}");
        let mut arch = ContainerBlueprint::new(kind)
            .with_contents(contents.iter().map(|c| IncludedContent(c.as_str().into())));
        match kind {
            ContainerKind::Horizontal => {
                arch = arch.with_col_shares(shares.iter().map(|s| ColumnShare((*s).into())));
            }
            ContainerKind::Vertical => {
                arch = arch.with_row_shares(shares.iter().map(|s| RowShare((*s).into())));
            }
            _ => {}
        }
        if let Some(active) = active {
            arch = arch.with_active_tab(ActiveTab(active.into()));
        }
        self.bp.log(path.as_str(), &arch).map_err(err)?;
        Ok((path, bytes))
    }
}

/// Sends the layout into `rec`, active at once. `chase`: there is an ego to
/// follow (`EGO_POSE` is logged).
pub fn send(
    rec: &RecordingStream,
    application_id: &'static str,
    recording_id: &str,
    chase: bool,
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
    let scenario = b.view("StateTimeline", "Scenario", "/scenario", &["$origin/**"])?;
    let speed = b.view("TimeSeries", "Speed", "/ego/speed_mps", &["$origin/**"])?;
    let control = b.view("TimeSeries", "Control", "/ego/control", &["$origin/**"])?;
    let result = b.view("TextDocument", "Result", "/result", &["$origin"])?;
    let log = b.view(
        "TextLog",
        "Log",
        "/",
        &[
            "/world/actors/events",
            "/world/traffic_lights/states",
            "/result/log",
            "/diagnostics/**",
        ],
    )?;
    let (tabs, _) = b.container(
        ContainerKind::Tabs,
        &[speed.clone(), control, result, log],
        &[],
        Some(&speed),
    )?;
    let (side, _) = b.container(
        ContainerKind::Vertical,
        &[scenario, tabs],
        &[1.0, 1.0],
        None,
    )?;
    let (_, root) = b.container(ContainerKind::Horizontal, &[scene, side], &[3.0, 2.0], None)?;

    let bp = &b.bp;
    bp.log(
        "viewport",
        &ViewportBlueprint::new()
            .with_root_container(RootContainer(root.into()))
            .with_auto_layout(false)
            .with_auto_views(false),
    )
    .map_err(err)?;
    for panel in ["blueprint_panel", "selection_panel"] {
        bp.log(
            panel,
            &PanelBlueprint::new().with_state(PanelState::Collapsed),
        )
        .map_err(err)?;
    }
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
