#ifndef BASE_CONTROLLER_H
#define BASE_CONTROLLER_H


unsigned int crc_chk(unsigned char* data, unsigned char length)
{
	int j;
	unsigned int reg_crc = 0xFFFF;

	while(length--)
	{
		reg_crc ^= *data++;
		for(j = 0; j < 8; j++)
		{
			if(reg_crc & 0x01)
			{
				reg_crc = (reg_crc >> 1)^0xA001;
			}
			else
			{
				reg_crc = (reg_crc >> 1);
			}
		}
	}
	return reg_crc;
}

#endif