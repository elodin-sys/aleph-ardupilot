{ writers }:

# ArduPilot's waf build calls `git rev-parse HEAD` to embed version info.
# In a Nix sandbox there's no .git, so we provide a fake git that returns
# a fixed revision string.  Based on lopsided98/ardupilot-flake.
writers.writePython3Bin "git" {} ''
    import argparse
    import sys


    def main() -> int:
        parser = argparse.ArgumentParser(prog="git")
        subparsers = parser.add_subparsers(title="subcommands")

        p = subparsers.add_parser("rev-parse")
        p.add_argument("rev", nargs="?")
        p.add_argument("--short", metavar="length", type=int)
        p.set_defaults(func=rev_parse)

        args = parser.parse_args()
        if hasattr(args, "func"):
            return args.func(args)
        return 1


    def rev_parse(args: argparse.Namespace) -> int:
        if not args.rev:
            return 0
        if args.rev != "HEAD":
            return 1
        rev = "0000000000000000000000000000000000000000"
        if args.short:
            rev = rev[: args.short]
        print(rev)
        return 0


    if __name__ == "__main__":
        sys.exit(main())
''
