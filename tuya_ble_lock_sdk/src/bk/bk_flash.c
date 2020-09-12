#include "bk_flash.h"




/*********************************************************************
 * LOCAL CONSTANTS
 */

/*********************************************************************
 * LOCAL STRUCT
 */

/*********************************************************************
 * LOCAL VARIABLES
 */

/*********************************************************************
 * LOCAL FUNCTION
 */

/*********************************************************************
 * VARIABLES
 */




/*********************************************************
FN: 
*/
void bk_flash_init(void)
{
}

/*********************************************************
FN: 
*/
void bk_flash_read(uint32_t addr, uint8_t *buf, uint32_t size)
{
    bk_wdt_feed();
    
    bk_enter_critical();
    flash_read(0, addr, size, buf, NULL);
    bk_exit_critical();
}

/*********************************************************
FN: 
*/
void bk_flash_write(uint32_t addr, uint8_t *buf, uint32_t size)
{
    bk_wdt_feed();
    
    bk_enter_critical();
    flash_write(0, addr, size, (void*)buf, NULL);
    bk_exit_critical();
}

/*********************************************************
FN: 
*/
void bk_flash_erase(uint32_t addr, uint32_t num)
{
    bk_wdt_feed();
    
    bk_enter_critical();
    flash_erase(0, addr, num*0x1000, NULL);
    bk_exit_critical();
}




















