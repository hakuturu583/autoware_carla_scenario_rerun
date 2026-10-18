"""The exceptions a conversion raises.

Defined in Python, not in the extension, so that they pickle: a conversion
run in a ``ProcessPoolExecutor`` worker raises the same exception, with its
``input``, ``path`` and ``line``, in the parent. Every attribute is also in
``args``, which is what pickling rebuilds an exception from.
"""

from __future__ import annotations


class ConversionError(RuntimeError):
    """A trajectory log could not be converted.

    Attributes:
        message: What went wrong.
        input: The trajectory log being converted.
        path: The file at fault: the log, or the ``_result.json``, the map or
            the recording it was reading or writing.
    """

    def __init__(self, message: str, input: str | None = None, path: str | None = None, *rest: object) -> None:
        RuntimeError.__init__(self, message, input, path, *rest)
        self.message = message
        self.input = input
        self.path = path

    def __str__(self) -> str:
        return self.message


class MalformedLineError(ConversionError):
    """A line of the trajectory log is not a record of the schema.

    Attributes:
        line: The line, 1-based.
    """

    def __init__(self, message: str, input: str | None = None, path: str | None = None, line: int | None = None) -> None:
        ConversionError.__init__(self, message, input, path, line)
        self.line = line


class UnknownRecordTypeError(ConversionError):
    """``strict=True`` and the log has a record type the schema does not define."""

    def __init__(
        self,
        message: str,
        input: str | None = None,
        path: str | None = None,
        line: int | None = None,
        record_type: str | None = None,
    ) -> None:
        ConversionError.__init__(self, message, input, path, line, record_type)
        self.line = line
        self.record_type = record_type


class UnknownFieldError(ConversionError):
    """``strict=True`` and a record has a field the schema does not define
    (``field``: ``record.field``, e.g. ``map.lanelet2.size``)."""

    def __init__(
        self,
        message: str,
        input: str | None = None,
        path: str | None = None,
        line: int | None = None,
        field: str | None = None,
    ) -> None:
        ConversionError.__init__(self, message, input, path, line, field)
        self.line = line
        self.field = field
