"""Validate the selected map files and prepare consistent Nav2 keepout filters."""

import argparse
from pathlib import Path
import sys

import yaml


def read_yaml(path):
    with Path(path).open(encoding='utf-8') as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError(f'YAML must contain a mapping: {path}')
    return value


def validate_map(path):
    path = Path(path)
    image = read_yaml(path).get('image')
    if not isinstance(image, str) or not image.strip():
        raise ValueError(f'Map YAML must specify an image: {path}')
    # Relative image paths belong to the YAML directory, regardless of extension.
    with (path.parent / image).open('rb'):
        pass


def is_keepout(settings):
    return (isinstance(settings, dict) and
            str(settings.get('plugin', '')).replace('/', '::').endswith('::KeepoutFilter'))


def prepare(map_file, mask_file, params_file, keepout, output):
    keepout = keepout.lower()
    keepout = {'1': 'true', '0': 'false'}.get(keepout, keepout)
    if keepout not in ('auto', 'true', 'false'):
        raise ValueError('use_keepout / USE_KEEPOUT must be auto, true or false')
    validate_map(map_file)
    if keepout == 'auto':
        # A missing optional mask disables keepout. A present but broken mask
        # is an error, so a damaged map cannot silently disable restrictions.
        keepout = 'true' if Path(mask_file).exists() else 'false'
    if keepout == 'true':
        validate_map(mask_file)

    params = read_yaml(params_file)
    for name in ('local_costmap', 'global_costmap'):
        costmap = params[name][name]['ros__parameters']
        filters = costmap.get('filters', [])
        if not isinstance(filters, list) or not all(isinstance(n, str) for n in filters):
            raise ValueError(f'{name}.filters must be a list of plugin names')
        keepout_names = [n for n in filters if is_keepout(costmap.get(n))]
        if keepout == 'true':
            if not keepout_names:
                # Do not overwrite a custom plugin using this name.
                if 'keepout_filter' in costmap and not is_keepout(costmap['keepout_filter']):
                    raise ValueError(f'{name}: keepout_filter names a different plugin')
                keepout_names = ['keepout_filter']
                filters = [*filters, 'keepout_filter']
                costmap.setdefault('keepout_filter', {
                    'plugin': 'nav2_costmap_2d::KeepoutFilter',
                    'filter_info_topic': '/costmap_filter_info',
                })
            for n in keepout_names:
                costmap[n]['enabled'] = True
        else:
            # Preserve unrelated filters, such as a custom speed filter.
            filters = [n for n in filters if n not in keepout_names]
        if filters:
            costmap['filters'] = filters
        else:
            costmap.pop('filters', None)
    with Path(output).open('w', encoding='utf-8') as stream:
        yaml.safe_dump(params, stream, allow_unicode=True, sort_keys=False)
    return keepout


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ('map', 'mask', 'params', 'keepout', 'output'):
        parser.add_argument('--' + name, required=True)
    args = parser.parse_args()
    try:
        result = prepare(args.map, args.mask, args.params, args.keepout, args.output)
    except (OSError, ValueError, KeyError, TypeError, yaml.YAMLError) as exc:
        print(f'Nav2 configuration error: {exc}', file=sys.stderr)
        return 1
    print(result)
    return 0


if __name__ == '__main__':
    sys.exit(main())
