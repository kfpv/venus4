#include <libpynq.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>

static int uart_initialized = 0;

void init_uart() {
    if (uart_initialized) return;
    
    switchbox_init();
    switchbox_set_pin(IO_AR0, SWB_UART0_RX);
    switchbox_set_pin(IO_AR1, SWB_UART0_TX);
    
    uart_init(UART0);
    uart_reset_fifos(UART0);
    uart_initialized = 1;
}

void send_message(const char* topic, const char* message) {
    if (!uart_initialized) init_uart();
        
    size_t topic_len = strlen(topic);
    size_t message_len = strlen(message);
    size_t total_len = topic_len + 1 + message_len;
    
    char* combined = malloc(total_len + 1);
    snprintf(combined, total_len + 1, "%s|%s", topic, message);
    
    uint32_t length = total_len;
    uint8_t* len_bytes = (uint8_t*)&length;
    
    printf("<< Outgoing Message: Size = %d\n", length);
    fflush(stdout); 
    
    for(uint32_t i = 0; i < 4; i++) {
        uart_send(UART0, len_bytes[i]); 
    }
    
    for(uint32_t i = 0; i < total_len; i++) {
        uart_send(UART0, combined[i]); 
    }
    
    free(combined);
}

int receive_message(char* topic_out, char* message_out, int max_len) {
    if (!uart_initialized) init_uart();
    
    if (!uart_has_data(UART0)) return 0;
    
    uint8_t read_len[4];
    for(uint32_t i = 0; i < 4; i++) {
        read_len[i] = uart_recv(UART0); 
    }
    uint32_t length = *((uint32_t*)read_len); 
    
    printf(">> Incoming Message: Length = %d\n", length);
    fflush(stdout); 
    
    if (length == 0 || length >= max_len) return 0;
    
    
    uint32_t i = 0;
    char* buffer = (char*) malloc(sizeof(char) * length);
    while(i < length) {
        buffer[i] = (char)uart_recv(UART0); 
        i++;
    }
    buffer[length] = '\0';
    
    
    char* separator = strchr(buffer, '|');
    if (!separator) {
        strcpy(topic_out, "default");
        strcpy(message_out, buffer);
    } else {
        *separator = '\0';
        strcpy(topic_out, buffer);
        strcpy(message_out, separator + 1);
    }
    
    printf("  >Topic: %s, Message: %s\n", topic_out, message_out);
    fflush(stdout); 
    
    free(buffer);
    return 1;
}

int has_data() {
    if (!uart_initialized) init_uart();
    return uart_has_data(UART0);
}

void cleanup_uart() {
    if (uart_initialized) {
        uart_reset_fifos(UART0);
        uart_destroy(UART0);
        pynq_destroy();
        uart_initialized = 0;
    }
}


int main(int argc, char* argv[]) {
    pynq_init();
    
    if (argc < 2) {
        printf("Usage: %s <command> [args...]\n", argv[0]);
        printf("Commands:\n");
        printf("  send <topic> <message>  - Send message\n");
        printf("  receive                 - Receive one message\n");
        printf("  listen                  - Listen continuously\n");
        printf("  daemon                  - Interactive daemon mode\n");
        printf("  hasdata                 - Check if data available\n");
        return 1;
    }
    
    
    if (strcmp(argv[1], "daemon") == 0) {
        printf("MQTT Bridge Daemon started\n");
        fflush(stdout);
        
        char line[16384];
        char topic[256], message[16128];
        
        while (fgets(line, sizeof(line), stdin)) {
            
            line[strcspn(line, "\n")] = 0;
            
            
            if (strstr(line, "|") != NULL) {
                
                char* separator = strchr(line, '|');
                *separator = '\0';
                strcpy(topic, line);
                strcpy(message, separator + 1);
                
                send_message(topic, message);
                
                
            }
            else if (strcmp(line, "RECEIVE") == 0) {
                if (receive_message(topic, message, 256)) {
                    printf("RECEIVED %s|%s\n", topic, message);
                } else {
                    printf("NO_DATA\n");
                }
                fflush(stdout);
            }
            else if (strcmp(line, "HASDATA") == 0) {
                printf("HASDATA %d\n", has_data());
                fflush(stdout);
            }
            else if (strcmp(line, "QUIT") == 0) {
                break;
            }
        }
        
        cleanup_uart();
        return 0;
    }
    
    if (strcmp(argv[1], "send") == 0 && argc == 4) {
        send_message(argv[2], argv[3]);
        printf("Sent: %s -> %s\n", argv[2], argv[3]);
    }
    else if (strcmp(argv[1], "receive") == 0) {
        char topic[256], message[256];
        if (receive_message(topic, message, 256)) {
            printf("%s|%s\n", topic, message);
        }
    }
    else if (strcmp(argv[1], "listen") == 0) {
        printf("Listening... Press Ctrl+C to stop\n");
        char topic[256], message[256];
        while (1) {
            if (receive_message(topic, message, 256)) {
                printf("%s|%s\n", topic, message);
                fflush(stdout);
            }
            usleep(1000); 
        }
    }
    else if (strcmp(argv[1], "hasdata") == 0) {
        printf("%d\n", has_data());
    }
    else {
        printf("Invalid command\n");
        return 1;
    }
    
    cleanup_uart();
    return 0;
}
