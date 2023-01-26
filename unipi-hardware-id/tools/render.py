from jinja2 import *

warning = "THIS FILE IS GENERATED FROM TEMPLATE. DON'T MODIFY IT"

def load_yaml_values(filename):
	import yaml

	with open(filename, 'r') as stream:
		data = yaml.load(stream, Loader=yaml.SafeLoader)
	return data
	#print(yaml.dump(data))

def gener_h(template, data):
	loader=FileSystemLoader(['.'])
	env = Environment(loader=loader)
	t = env.get_template(name=template)

	return t.render(warning=warning, **data)


if __name__ == "__main__":
	import argparse, sys
	a = argparse.ArgumentParser(prog='expand_template.py')
	a.add_argument('template', metavar="file", help='input template', type=str, nargs=1)
	a.add_argument('-d','--data', metavar="file", help='use this file as data yaml')
	a.add_argument('-o','--output', metavar="file", help='write result to file')
	args = a.parse_args()
	datafile = args.data if args.data else "values/uniee_values.yaml"

	data = load_yaml_values(datafile)
	result = gener_h(args.template[0], data)

	if args.output:
		with open(args.output, 'w') as f:
			f.write(result)
	else:
		sys.stdout.write(result)
