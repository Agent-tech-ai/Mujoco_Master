"""Compile and validate both local simulator variants."""

from master_sim.model import load_model, validate_model, validation_summary


def main() -> int:
    exit_code = 0
    for free_base in (False, True):
        model = load_model(free_base=free_base)
        print(f"=== {'FREE BASE' if free_base else 'FIXED BASE'} ===")
        print(validation_summary(model))
        if validate_model(model):
            exit_code = 1
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())

