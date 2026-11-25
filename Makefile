# Documentation only; the code builds with docker compose (Documentation/build.rst).

VENV := .venv
DOCS_OUT := $(VENV)/docs-build

.PHONY: htmldocs livedocs

$(VENV)/bin/sphinx-autobuild:
	uv venv --allow-existing $(VENV)
	uv pip install --python $(VENV)/bin/python sphinx sphinx-autobuild

# One-off build: open .venv/docs-build/dirhtml/index.html
htmldocs: $(VENV)/bin/sphinx-autobuild
	$(VENV)/bin/sphinx-build -M dirhtml Documentation $(DOCS_OUT)

# Live preview at http://127.0.0.1:8000, rebuilt on every save
livedocs: $(VENV)/bin/sphinx-autobuild
	$(VENV)/bin/sphinx-autobuild -b dirhtml Documentation $(DOCS_OUT)/live
