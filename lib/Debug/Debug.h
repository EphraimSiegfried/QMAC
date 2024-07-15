// Only includes serial statements if compiled with DEBUG flag

#pragma once

#ifdef DEBUG
#define LOG(str) Serial.println(str)
#else
#endif
