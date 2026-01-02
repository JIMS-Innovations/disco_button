#include "PiPlates.h" // Include the PiPlates C library header
#include <unistd.h>   // For the sleep function
#include <stdio.h>    // For printf

int main() {
    int k;
    
    // Initialize the Pi-Plates system
    PiPlateInitialize(); 

    // Loop through all 7 (RELAYplate) or 8 (RELAYplate2) relays
    // Note: Original RELAYplate has 7 relays, RELAYplate2 has 8
    for(k = 1; k < 7; ++k) { 
        printf("Toggling Relay %d ON\\n", k);
        // Turn on the specific relay (address 1, relay number k)
        RelayBoard_relayON(1, k); 
        sleep(1); // Wait for 1 second

        printf("Toggling Relay %d OFF\\n", k);
        // Turn off the specific relay (address 1, relay number k)
        RelayBoard_relayOFF(1, k); 
        sleep(1); // Wait for 1 second
    }

    printf("Complete\\n");
    return 0;
}
