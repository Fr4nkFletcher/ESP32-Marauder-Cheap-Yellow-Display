# simple script to update all examples' flash_config.h based on the file in this folder
from pathlib import Path
import click

import shutil

@click.command()
@click.argument('dir', required=True)
def main(dir):
    """
    This script takes a mandatory 'dir' argument, which is a path to pivot example to update for all DualRole's examples
    """
    examples_dir = Path('examples')
    sample_dir = examples_dir / dir
    assert sample_dir.is_dir(), f"examples/{dir} does not exist or is not a valid dir."
    sample_file = sample_dir / 'flash_config.h'

    f_list = sorted(examples_dir.glob('*/flash_config.h'))
    for f in f_list:
        if f != sample_file:
            click.echo(f"Updating {f}")
            shutil.copy(sample_file, f)

if __name__ == '__main__':
    main()
