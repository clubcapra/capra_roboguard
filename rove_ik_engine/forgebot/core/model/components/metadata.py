"""Metadata component: free-form key/value notes attached to any entity."""

from __future__ import annotations

from typing import Any

from pydantic import Field

from .base import BaseComponent, register_component


@register_component("metadata")
class MetadataComponent(BaseComponent):
    tags: list[str] = Field(default_factory=list)
    notes: str = ""
    data: dict[str, Any] = Field(default_factory=dict)
