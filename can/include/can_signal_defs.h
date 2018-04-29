#include <string>
#include <stdbool.h>

typedef struct {
	int start_bit;
	int bit_length;
	bool is_signed;
	bool big_endian;
	double scaling;
	double offset;
	int can_id;
	std::string units;
} signal_t;

enum {
	rel_lane,
	range_rate,
	obj_id,
	lat_range,
	brake_light,
	long_range,
	azimuth,
	obj_age,
	adas_signal_count
} adas_signals;

enum {
	vehicle_speed,
	gmhs_signal_count
} gmhs_signals;

signal_t adas_signal_defs[adas_signal_count] = {
	{
		10,
		2,
		false,
		true,
		1,
		0
	},
	{
		12,
		12,
		true,
		true,
		0.0625,
		0
	},
	{
		24,
		6,
		false,
		true,
		1,
		0
	},
	{
		30,
		10,
		true,
		true,
		0.125,
		0
	},
	{
		42,
		2,
		false,
		true,
		1,
		0
	},
	{
		44,
		12,
		false,
		true,
		0.1,
		0
	},
	{
		48,
		10,
		true,
		true,
		0.1,
		0
	},
	{
		57,
		7,
		false,
		true,
		1,
		0
	}
};

signal_t gmhs_signal_defs[gmhs_signal_count] = {
	{	
		8,
		15,
		false,
		true,
		0.01562,
		0,
		0x3E9
	}
};

int adas_msg_ids[3] = {0x441, 0x442, 0x443};


