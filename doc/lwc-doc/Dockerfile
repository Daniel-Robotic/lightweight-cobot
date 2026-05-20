FROM squidfunk/mkdocs-material
RUN apk add --no-cache \
    cairo \
    pango \
    gdk-pixbuf \
    libffi \
    fontconfig \
    ttf-dejavu \
    gcc \
    g++ \
    musl-dev \
    python3-dev \
    && pip install mkdocs-with-pdf \
    && apk del --no-cache gcc g++ musl-dev python3-dev
