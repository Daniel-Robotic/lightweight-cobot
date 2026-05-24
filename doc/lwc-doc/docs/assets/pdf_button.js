document.addEventListener("DOMContentLoaded", function () {
  const btn = document.createElement("a");
  btn.href = "/pdf/document.pdf";
  btn.download = "lwc-documentation.pdf";
  btn.title = "Скачать всю документацию в PDF";
  btn.style.cssText = [
    "position: fixed",
    "bottom: 24px",
    "right: 24px",
    "z-index: 9999",
    "background: var(--md-primary-fg-color, #1976d2)",
    "color: #fff",
    "padding: 12px 18px",
    "border-radius: 24px",
    "text-decoration: none",
    "font-weight: 600",
    "font-size: 14px",
    "box-shadow: 0 3px 10px rgba(0,0,0,0.25)",
    "display: flex",
    "align-items: center",
    "gap: 8px",
    "transition: opacity .2s",
  ].join(";");
  btn.innerHTML = "&#128196; Скачать PDF";
  btn.onmouseenter = () => (btn.style.opacity = "0.85");
  btn.onmouseleave = () => (btn.style.opacity = "1");
  document.body.appendChild(btn);
});
