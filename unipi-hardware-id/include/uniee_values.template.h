/******************************************************************
 *
 *  {{ warning }}
 *
 * uniee_values.h
 *
 *  Created on: Jan 14, 2022
 *      Author: Miroslav Ondra <ondra@faster.cz>
 * 
 */

#ifndef UNIEE_VALUES_H_
#define UNIEE_VALUES_H_

/* uniee_bank_3_t.platform_id */
{% for f,v in platform.family.items() %} #define UNIEE_PLATFORM_FAMILY_{{ f|upper }}   {{ "0x%02x" % v }}
{% endfor %}

{% for f,v in platform.id.items() %} #define UNIEE_PLATFORM_ID_{{ f|upper }}   {{ "0x%04x" % v }}
{% endfor %}

/* uniee_bank_2_t.board_model */
{% for m,v in board.model.items() %} #define UNIEE_BOARD_MODEL_{{ m|upper }}	{{ v }}
{% endfor %}

/* specdata_header_t.field_type */
{% for t,v in field.type.items() %} #define UNIEE_FIELD_TYPE_{{ t|upper }}	{{ v }}
{% endfor %}

/* values of fields stored in uniee_bank_1_t.dummy_data */
{% for t,d in field.value.items() %}
{% for n,v in d.items() %} #define UNIEE_FIELD_VALUE_{{ t|upper }}_{{ n|upper }}	{{ v }}
{% endfor %}
{% endfor %}

#endif
