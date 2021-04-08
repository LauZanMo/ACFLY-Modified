#pragma once

#include "Modes.hpp"

class M32_PosCtrl:public Mode_Base 
{
	private:
		
	public:
		M32_PosCtrl();
		virtual void get_MavlinkMode( ModeFuncCfg cfg, Receiver rc, 
																	uint8_t btn_zones[4], AFunc* mode );
		virtual ModeResult main_func( void* param1, uint32_t param2 );
};