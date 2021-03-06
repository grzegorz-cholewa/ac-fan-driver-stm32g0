#include <config.h>
#include <logger.h>
#include <sensor.h>

/* STATIC FUNCTIONS DECLARATIONS */
/* converts 10bit ADC read value to temperature in Celsius degree */
int16_t ntc_to_temperature(uint16_t adc_value);


// lookup table for temperature reads; index is ADC value and values are in Celcius Degree * 10
// this table is for: NTC 1k@25C with 200Ohm pull-up, beta = 3730
int NTC_table[513] = {
	6302, 5265, 4228, 3736, 3427, 3206, 3037,
	2901, 2788, 2692, 2608, 2535, 2470, 2411,
	2358, 2309, 2264, 2222, 2184, 2148, 2114,
	2082, 2052, 2024, 1997, 1971, 1947, 1924,
	1901, 1880, 1860, 1840, 1821, 1803, 1785,
	1769, 1752, 1736, 1721, 1706, 1692, 1678,
	1664, 1651, 1638, 1625, 1613, 1601, 1589,
	1578, 1567, 1556, 1546, 1535, 1525, 1515,
	1505, 1496, 1486, 1477, 1468, 1459, 1451,
	1442, 1434, 1425, 1417, 1409, 1402, 1394,
	1386, 1379, 1371, 1364, 1357, 1350, 1343,
	1336, 1330, 1323, 1316, 1310, 1304, 1297,
	1291, 1285, 1279, 1273, 1267, 1261, 1255,
	1250, 1244, 1238, 1233, 1227, 1222, 1217,
	1211, 1206, 1201, 1196, 1191, 1186, 1181,
	1176, 1171, 1166, 1161, 1157, 1152, 1147,
	1143, 1138, 1133, 1129, 1125, 1120, 1116,
	1111, 1107, 1103, 1098, 1094, 1090, 1086,
	1082, 1078, 1074, 1070, 1066, 1062, 1058,
	1054, 1050, 1046, 1042, 1038, 1035, 1031,
	1027, 1023, 1020, 1016, 1012, 1009, 1005,
	1002, 998, 995, 991, 988, 984, 981, 977,
	974, 970, 967, 964, 960, 957, 954, 950, 947,
	944, 941, 937, 934, 931, 928, 925, 921, 918,
	915, 912, 909, 906, 903, 900, 897, 894, 891,
	888, 885, 882, 879, 876, 873, 870, 867, 864,
	861, 858, 856, 853, 850, 847, 844, 841, 839,
	836, 833, 830, 827, 825, 822, 819, 816, 814,
	811, 808, 805, 803, 800, 797, 795, 792, 789,
	787, 784, 781, 779, 776, 774, 771, 768, 766,
	763, 761, 758, 755, 753, 750, 748, 745, 743,
	740, 738, 735, 732, 730, 727, 725, 722, 720,
	717, 715, 712, 710, 707, 705, 702, 700, 698,
	695, 693, 690, 688, 685, 683, 680, 678, 676,
	673, 671, 668, 666, 663, 661, 659, 656, 654,
	651, 649, 647, 644, 642, 639, 637, 635, 632,
	630, 627, 625, 623, 620, 618, 615, 613, 611,
	608, 606, 604, 601, 599, 596, 594, 592, 589,
	587, 585, 582, 580, 577, 575, 573, 570, 568,
	566, 563, 561, 558, 556, 554, 551, 549, 547,
	544, 542, 539, 537, 535, 532, 530, 527, 525,
	523, 520, 518, 515, 513, 511, 508, 506, 503,
	501, 499, 496, 494, 491, 489, 486, 484, 482,
	479, 477, 474, 472, 469, 467, 464, 462, 459,
	457, 454, 452, 449, 447, 444, 442, 439, 437,
	434, 432, 429, 427, 424, 422, 419, 416, 414,
	411, 409, 406, 403, 401, 398, 396, 393, 390,
	388, 385, 382, 380, 377, 374, 372, 369, 366,
	363, 361, 358, 355, 352, 349, 347, 344, 341,
	338, 335, 332, 330, 327, 324, 321, 318, 315,
	312, 309, 306, 303, 300, 297, 294, 291, 288,
	284, 281, 278, 275, 272, 269, 265, 262, 259,
	256, 252, 249, 246, 242, 239, 235, 232, 228,
	225, 221, 218, 214, 210, 207, 203, 199, 195,
	192, 188, 184, 180, 176, 172, 168, 164, 160,
	155, 151, 147, 143, 138, 134, 129, 125, 120,
	115, 110, 106, 101, 96, 91, 86, 80, 75, 70,
	64, 59, 53, 47, 41, 35, 29, 22, 16, 9, 3,
	-4, -12, -19, -27, -34, -42, -51, -59, -68,
	-77, -87, -97, -107, -118, -129, -142, -154,
	-168, -182, -197, -214, -232, -252, -274,
	-299, -328, -363, -405, -463, -555, -647
};


int16_t ntc_to_temperature(uint16_t adc_value)
{
	int16_t tableIdx = adc_value/2;
	if (tableIdx < sizeof(NTC_table)/sizeof(int))
	{
		return NTC_table[adc_value/2]; // table consists of 512 elements, while 10-bit ADC have max value 1024
	}
	else
	{
		return -999; // TODO: solve this better way
	}
}

int16_t pt100_to_temperature(uint16_t adc_value){
//	return PTC_table[adc_value];
	 return 0; // TODO get real values
}


int16_t check_for_error(sensors_t * sensor_array)
{
	int16_t ret_val = 0;
	for (int i = 0; i < TOTAL_SENSOR_NUMBER; i++)
	{
		volatile int16_t temperature = sensor_array[i].temperature;
		// check if sensor is set as connected and if temperature read is unexpected.
		if ((temperature < MIN_WORKING_TEMPERATURE) || (temperature > MAX_WORKING_TEMPERATURE))
		{
			// if sensor is configured as connected but temperature read is incorrect, modify return value to indicate error
			if (sensor_array[i].connected_status)
			{
				sensor_array[i].error = 1;
				ret_val |= 1 << i; // if error is detected on given 'i' channel, set proper bit to 1 to indicate error on that channel
				logger_log(LEVEL_ERROR, "Temperature out of range on ch %d\r\n", i+1); // i counts from 0, sensors are counted from 1
			}
		}
		else
		{
			sensor_array[i].error = 0;
		}
	}

	return ret_val;
}


