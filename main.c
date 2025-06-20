#include <libpynq.h>
#include <stepper.h>

int main() {
    pynq_init();
    stepper_init();
    stepper_enable();
        
    return 0;
}
