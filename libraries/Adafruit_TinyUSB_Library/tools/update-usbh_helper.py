from pathlib import Path
import click
import sys
import shutil

@click.command()
@click.argument('dir', type=click.Path(), required=True)
def main(dir):
    """
    This script takes a mandatory 'dir' argument, which is a path to pivot example to update for all DualRole's examples
    """
    sample_dir = Path(dir)
    if not sample_dir.is_dir():
        # add examples/DualRoles to the path
        sample_dir = Path('examples/DualRole') / sample_dir
        if not sample_dir.is_dir():
            click.echo(f"The specified dir '{dir}' does not exist or is not a valid dir.")
            sys.exit(1)

    sample_file = sample_dir / 'usbh_helper.h'
    f_list = sorted(Path('examples/DualRole').glob('**/usbh_helper.h'))
    for f in f_list:
        if f != sample_file:
            click.echo(f"Updating {f}")
            shutil.copy(sample_file, f)


if __name__ == '__main__':
    main()