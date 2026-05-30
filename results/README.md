# Results

This directory contains historical benchmark reports and generated validation artifacts.

For a clean public repository, prefer keeping:

- compact Markdown reports
- summary CSV files
- representative figures used in documentation

Avoid committing:

- large iterative JSON sweeps
- generated PDFs that can be rebuilt
- temporary experiment outputs
- private or unpublished dataset artifacts

Recommended public-facing summaries:

- `engineering_validation/engineering_validation_report.md`
- `public_dataset_test/public_dataset_validation_report.md`
- `layer_validation/validation_report_2026-05-08.md`
- `health_observation_adaptive_switch_report.md`

Large or repeatedly generated result sets should be published as release artifacts or external archives and linked from `docs/validation_demo.md`.
