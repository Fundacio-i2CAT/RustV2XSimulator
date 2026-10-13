# Contributing to RustV2XSimulator

Thank you for your interest in contributing to **`rustv2xsimulator`**!

We welcome bug reports, feature requests, documentation improvements, and code contributions from the research and open-source communities.

---

## 1. Code of Conduct

Please be respectful and constructive in all issues, pull requests, and community discussions.

---

## 2. Reporting Issues

When filing an issue:
- **Bug Reports**: Include your operating system, Rust version (`rustc --version`), SUMO version (if applicable), steps to reproduce the issue, and terminal error logs.
- **Feature Requests**: Describe the use case and context (e.g., a new wireless technology model, protocol extension, or mobility interface).

---

## 3. Development Workflow

### Prerequisites
- Stable Rust toolchain ($\ge 1.85$). Install via [rustup.rs](https://rustup.rs).
- `libpcap-dev` (Linux) or equivalent for packet capture export.
- (Optional) Eclipse SUMO ($\ge 1.18$) for TraCI mobility tests.

### Building
```bash
cargo build --release
```

### Running Tests
All contributions must pass the complete unit and integration test suite:
```bash
cargo test --all-targets --all-features
```

### Code Formatting & Linting
Ensure code adheres to standard Rust formatting and clippy checks:
```bash
cargo fmt --check
cargo clippy --all-targets
```

### Documentation
If your changes update public APIs or architecture:
- Verify Rust API documentation builds without warnings:
  ```bash
  cargo doc --no-deps
  ```
- If modifying documentation in `docs/`, verify that MkDocs builds cleanly:
  ```bash
  pip install -r docs/requirements.txt
  mkdocs build --strict
  ```

---

## 4. Submitting Pull Requests

1. Fork the repository and create a feature branch (`git checkout -b feature/my-feature`).
2. Commit your changes with descriptive commit messages.
3. Verify that all tests pass (`cargo test --all-targets --all-features`).
4. Push to your branch and open a Pull Request.

---

## 5. License

By contributing to `rustv2xsimulator`, you agree that your contributions will be licensed under the project's [GNU Affero General Public License v3.0 (AGPL-3.0)](LICENSE).
