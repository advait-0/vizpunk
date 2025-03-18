#include <stdio.h>
#include <stdint.h>
#include <math.h>

// Define wave commands
#define SINE_WAVE     0x0000
#define TRIANGLE_WAVE 0x0002
#define SQUARE_WAVE   0x0020  // MSB output
#define SQUARE_WAVE_DIV2 0x0008  // MSB/2 output

// Byte swap function
uint16_t swap_bytes(uint16_t value) {
    return ((value & 0xFF00) >> 8) | ((value & 0x00FF) << 8);
}

// Calculate frequency registers with integrated byte swapping
void calculate_frequency_registers(float freq_hz, uint16_t *freq_lsb, uint16_t *freq_msb) {
    uint32_t freq_word = (uint32_t)((freq_hz * pow(2, 28)) / 25000000.0);
    
    // Extract and format LSB with '01' prefix
    uint16_t raw_lsb = (freq_word & 0x3FFF) | 0x4000;
    *freq_lsb = swap_bytes(raw_lsb);
    
    // Extract and format MSB with '10' prefix
    uint16_t raw_msb = ((freq_word >> 14) & 0x3FFF) | 0x4000;
    *freq_msb = swap_bytes(raw_msb);
}

int main() {
    float frequency;
    int wave_type;
    uint16_t reset_cmd = 0x2100;
    uint16_t exit_reset = 0x2002;
    uint16_t freq_lsb, freq_msb, wave_cmd;

    printf("Enter frequency (Hz): ");
    scanf("%f", &frequency);
    
    printf("Select wave type:\n");
    printf("1. Sine Wave\n");
    printf("2. Triangle Wave\n");
    printf("3. Square Wave (MSB Output)\n");
    printf("4. Square Wave (MSB/2 Output)\n");
    printf("Enter choice: ");
    scanf("%d", &wave_type);
    
    // Set wave command based on user selection
    switch (wave_type) {
        case 1: wave_cmd = swap_bytes(SINE_WAVE); break;
        case 2: wave_cmd = swap_bytes(TRIANGLE_WAVE); break;
        case 3: wave_cmd = swap_bytes(SQUARE_WAVE); break;
        case 4: wave_cmd = swap_bytes(SQUARE_WAVE_DIV2); break;
        default: 
            printf("Invalid choice!\n");
            return 1;
    }

    wave_cmd = wave_cmd | exit_reset;
    swap_bytes(wave_cmd);
    
    // Calculate frequency registers
    calculate_frequency_registers(frequency, &freq_lsb, &freq_msb);
    
    // Print calculated values
    printf("\nReset Command: 0x%04X\n", swap_bytes(reset_cmd));
    printf("Frequency LSB: 0x%04X\n", freq_lsb);
    printf("Frequency MSB: 0x%04X\n", freq_msb);
    printf("Wave Command: 0x%04X\n", wave_cmd);
    
    return 0;
}
