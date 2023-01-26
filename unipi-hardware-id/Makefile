
VALUES_YAML = values/uniee_values.yaml


include/uniee_values.h: include/uniee_values.template.h $(VALUES_YAML)
	@python3 tools/render.py -d $(VALUES_YAML) -o $@ include/uniee_values.template.h


all: include/uniee_values.h
