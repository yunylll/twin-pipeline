"""The one exception the MJCF builder raises; the stage turns it into StageResult.failed."""

from __future__ import annotations


class BuildError(Exception):
    def __init__(self, code: str, message: str, more: list["BuildError"] | None = None):
        super().__init__(message)
        self.code = code
        self.message = message
        self.more = list(more or [])

    def rows(self) -> list[dict]:
        return [{"code": error.code, "message": error.message} for error in self.more]
