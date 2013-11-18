/*
 * rfm23bp_lookup.c
 *
 *  Created on: Nov 8, 2013
 *      Author: andki
 */

#define CREATE_REGISTER_TABLE
#include "rfm23bp_table.h"
#undef CREATE_REGISTER_TABLE

uint8_t RFM23BPLOOKUP__GetAddress(uint8_t address)
{
	return rfm23bp_address_table[address].address;
}

