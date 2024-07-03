
VALUES_YAML = values/uniee_values.yaml

all: clean include/uniee_values.h

include/uniee_values.h: include/uniee_values.template.h $(VALUES_YAML)
	@python3 tools/render.py -d $(VALUES_YAML) -o $@ include/uniee_values.template.h

clean:
	@rm -f include/uniee_values.h
