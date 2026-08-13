"""Generate the shared PDF only during the default-language build."""

from mkdocs.plugins import event_priority


@event_priority(-101)
def on_config(config):
    """Disable mkdocs-to-pdf for non-default i18n build passes."""
    i18n = config.plugins.get("i18n")
    pdf = config.plugins.get("to-pdf")

    if i18n is not None and pdf is not None:
        pdf.enabled = i18n.current_language == i18n.default_language

    return config
