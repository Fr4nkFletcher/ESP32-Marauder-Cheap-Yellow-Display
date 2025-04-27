#if !defined(LITTLE_FOOT_PRINT)

#include "Arduino_G.h"

/**************************************************************************/
/*!
   @brief    Instatiate a GFX context for graphics! Can only be done by a superclass
   @param    w   Display width, in pixels
   @param    h   Display height, in pixels
*/
/**************************************************************************/
Arduino_G::Arduino_G(int16_t w, int16_t h) : WIDTH(w), HEIGHT(h)
{
}

#endif // !defined(LITTLE_FOOT_PRINT)
