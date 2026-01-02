#include "PiPlates.h"


int main()
{
    int k;
    char RelayState;
    int RelayHandle;

    gpioInitialise();
    PiPlateInitialize();



    for(k=1;k<7;++k)
    {
        /// Testing the relayON and RelayOFF functions ///
//        RelayBoard_relayON(1,k);
//        sleep(1);
//        RelayBoard_relayOFF(1,k);
//        sleep(1);

        /// Testing the relayTOGGLE function ///
        RelayBoard_relayTOGGLE(1,k);
        sleep(1);

        /// Testing the relayALL function ///
//        RelayBoard_relayALL(1,k+3);
//        sleep(1);

        /// Testing the LED functions ///
//        RelayBoard_setLED(1);
//        sleep(1);
//        RelayBoard_clrLED(1);
//        sleep(1);

//        RelayBoard_toggleLED(1);
//        sleep(1);
    }

    /// Testing the RelayBoard_relaySTATE function ///
    RelayState = RelayBoard_relaySTATE(1);
    printf("RelayState: %i\n", RelayState);
    sleep(1);

    gpioTerminate();

    printf("Complete\n");
    return 0;
}
