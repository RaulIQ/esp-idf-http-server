/* Simple HTTP Server Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <sys/param.h>
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "protocol_examples_utils.h"
#include "esp_tls_crypto.h"
#include <esp_http_server.h>
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_tls.h"
#include "esp_check.h"
#include "led_strip.h"
#include "esp_spiffs.h"

#if !CONFIG_IDF_TARGET_LINUX
#include <esp_wifi.h>
#include <esp_system.h>
#include "nvs_flash.h"
#include "esp_eth.h"
#endif  // !CONFIG_IDF_TARGET_LINUX

#define EXAMPLE_HTTP_QUERY_KEY_MAX_LEN  (64)
// #define BLINK_GPIO CONFIG_BLINK_GPIO
#define BLINK_GPIO 10

static bool s_led_state = false;

/* A simple example that demonstrates how to create GET and POST
 * handlers for the web server.
 */

static const char *TAG = "example";

static const char html_response[] = "<!DOCTYPE html><html><head><title>Hello</title></head>"
                                    "<body><h1>Hello, World!</h1></body></html>";
                                    
typedef struct {
    uint8_t r;
    uint8_t g;
    uint8_t b;
} rgb_t;

static rgb_t s_color = {128, 64, 255}; 

static led_strip_handle_t led_strip;

static void blink_led(void)
{
    /* If the addressable LED is enabled */
    if (s_led_state) {
        /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
        led_strip_set_pixel(led_strip, 0, s_color.g, s_color.r, s_color.b);
        /* Refresh the strip to send data */
        led_strip_refresh(led_strip);
    } else {
        /* Set all LED off to clear all pixels */
        led_strip_clear(led_strip);
    }
}

static void configure_led(void)
{
    ESP_LOGI(TAG, "Example configured to blink addressable LED!");
    /* LED strip initialization with the GPIO and pixels number*/
    led_strip_config_t strip_config = {
        .strip_gpio_num = BLINK_GPIO,
        .max_leds = 1, // at least one LED on board
    };
    led_strip_rmt_config_t rmt_config = {
        .resolution_hz = 10 * 1000 * 1000, // 10MHz
        .flags.with_dma = false,
    };
    ESP_ERROR_CHECK(led_strip_new_rmt_device(&strip_config, &rmt_config, &led_strip));
    /* Set all LED off to clear all pixels */
    led_strip_clear(led_strip);
}

static char* replace_placeholders(const char* line) {
    const char* placeholders[] = {"{{R}}", "{{G}}", "{{B}}", "{{STATE}}", "{{BUTTON_TEXT}}"};
    char r_str[4], g_str[4], b_str[4];
    snprintf(r_str, sizeof(r_str), "%d", s_color.r);
    snprintf(g_str, sizeof(g_str), "%d", s_color.g);
    snprintf(b_str, sizeof(b_str), "%d", s_color.b);
    const char* state_str = s_led_state ? "On" : "Off";
    const char* button_text_str = s_led_state ? "Turn Off" : "Turn On";
    const char* values[] = {r_str, g_str, b_str, state_str, button_text_str};

    char* result = strdup(line);
    if (!result) return NULL;

    for (int i = 0; i < 5; i++) {
        char* pos;
        while ((pos = strstr(result, placeholders[i]))) {
            size_t len_before = pos - result;
            size_t len_value = strlen(values[i]);
            size_t len_after = strlen(pos + strlen(placeholders[i]));
            char* new_result = malloc(len_before + len_value + len_after + 1);
            if (!new_result) {
                free(result);
                return NULL;
            }
            memcpy(new_result, result, len_before);
            memcpy(new_result + len_before, values[i], len_value);
            memcpy(new_result + len_before + len_value, pos + strlen(placeholders[i]), len_after);
            new_result[len_before + len_value + len_after] = '\0';
            free(result);
            result = new_result;
        }
    }
    return result;
}

/* An HTTP GET handler */
static esp_err_t hello_get_handler(httpd_req_t *req)
{
    char*  buf;
    size_t buf_len;

    /* Get header value string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_hdr_value_len(req, "Host") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        ESP_RETURN_ON_FALSE(buf, ESP_ERR_NO_MEM, TAG, "buffer alloc failed");
        /* Copy null terminated value string into buffer */
        if (httpd_req_get_hdr_value_str(req, "Host", buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found header => Host: %s", buf);
        }
        free(buf);
    }

    buf_len = httpd_req_get_hdr_value_len(req, "Test-Header-2") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        ESP_RETURN_ON_FALSE(buf, ESP_ERR_NO_MEM, TAG, "buffer alloc failed");
        if (httpd_req_get_hdr_value_str(req, "Test-Header-2", buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found header => Test-Header-2: %s", buf);
        }
        free(buf);
    }

    buf_len = httpd_req_get_hdr_value_len(req, "Test-Header-1") + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        ESP_RETURN_ON_FALSE(buf, ESP_ERR_NO_MEM, TAG, "buffer alloc failed");
        if (httpd_req_get_hdr_value_str(req, "Test-Header-1", buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found header => Test-Header-1: %s", buf);
        }
        free(buf);
    }

    /* Read URL query string length and allocate memory for length + 1,
     * extra byte for null termination */
    buf_len = httpd_req_get_url_query_len(req) + 1;
    if (buf_len > 1) {
        buf = malloc(buf_len);
        ESP_RETURN_ON_FALSE(buf, ESP_ERR_NO_MEM, TAG, "buffer alloc failed");
        if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
            ESP_LOGI(TAG, "Found URL query => %s", buf);
            char param[EXAMPLE_HTTP_QUERY_KEY_MAX_LEN], dec_param[EXAMPLE_HTTP_QUERY_KEY_MAX_LEN] = {0};
            /* Get value of expected key from query string */
            if (httpd_query_key_value(buf, "query1", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => query1=%s", param);
                example_uri_decode(dec_param, param, strnlen(param, EXAMPLE_HTTP_QUERY_KEY_MAX_LEN));
                ESP_LOGI(TAG, "Decoded query parameter => %s", dec_param);
            }
            if (httpd_query_key_value(buf, "query3", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => query3=%s", param);
                example_uri_decode(dec_param, param, strnlen(param, EXAMPLE_HTTP_QUERY_KEY_MAX_LEN));
                ESP_LOGI(TAG, "Decoded query parameter => %s", dec_param);
            }
            if (httpd_query_key_value(buf, "query2", param, sizeof(param)) == ESP_OK) {
                ESP_LOGI(TAG, "Found URL query parameter => query2=%s", param);
                example_uri_decode(dec_param, param, strnlen(param, EXAMPLE_HTTP_QUERY_KEY_MAX_LEN));
                ESP_LOGI(TAG, "Decoded query parameter => %s", dec_param);
            }
        }
        free(buf);
    }

    httpd_resp_set_type(req, "text/html");

    /* Set some custom headers */
    httpd_resp_set_hdr(req, "Custom-Header-1", "Custom-Value-1");
    httpd_resp_set_hdr(req, "Custom-Header-2", "Custom-Value-2");

    /* Send response with custom headers and body set as the
     * string passed in user context*/
    // const char* resp_str = (const char*) req->user_ctx;
    httpd_resp_send(req, html_response, HTTPD_RESP_USE_STRLEN);

    /* After sending the HTTP response the old HTTP request
     * headers are lost. Check if HTTP request headers can be read now. */
    if (httpd_req_get_hdr_value_len(req, "Host") == 0) {
        ESP_LOGI(TAG, "Request headers lost");
    }
    return ESP_OK;
}

static const httpd_uri_t hello = {
    .uri       = "/hello",
    .method    = HTTP_GET,
    .handler   = hello_get_handler,
    /* Let's pass response string in user
     * context to demonstrate it's usage */
    .user_ctx  = NULL
};

static esp_err_t root_get_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");

    FILE* f = fopen("/www/index.html", "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    char buffer[256];
    while (fgets(buffer, sizeof(buffer), f)) {
        if (strstr(buffer, "{{NAME}}")) {
            const char *name = "Masha";
            httpd_resp_send_chunk(req, name, strlen(name));
        } else {
            httpd_resp_send_chunk(req, buffer, strlen(buffer));
        }
    }
    fclose(f);

    httpd_resp_send_chunk(req, NULL, 0);

    return ESP_OK;
}

static const httpd_uri_t root = {
    .uri       = "/",              // корень сайта
    .method    = HTTP_GET,
    .handler   = root_get_handler,
    .user_ctx  = NULL
};

static esp_err_t light_control_handler(httpd_req_t *req) {
    FILE* f = fopen("/www/light_control.html", "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    if (req->method == HTTP_POST) {
        s_led_state = !s_led_state;
        blink_led(); // Assuming this toggles the light
        ESP_LOGI(TAG, "Light turned %s", s_led_state ? "on" : "off");
        httpd_resp_set_status(req, "303 See Other");
        httpd_resp_set_hdr(req, "Location", "/light");
        httpd_resp_send(req, NULL, 0);
        fclose(f);
        return ESP_OK;
    } else { // HTTP_GET
        httpd_resp_set_type(req, "text/html");
        char buffer[256];
        while (fgets(buffer, sizeof(buffer), f)) {
            char* replaced = replace_placeholders(buffer);
            if (replaced) {
                httpd_resp_send_chunk(req, replaced, strlen(replaced));
                free(replaced);
            } else {
                httpd_resp_send_chunk(req, buffer, strlen(buffer));
            }
        }
        fclose(f);
        httpd_resp_send_chunk(req, NULL, 0);
        return ESP_OK;
    }
}

static esp_err_t color_control_handler(httpd_req_t *req)
{
    char buf[100];
    int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }
    buf[ret] = '\0';

    // Parse form data (e.g., "r=100&g=150&b=200")
    char *ptr = buf;
    while (ptr != NULL) {
        char *key = strsep(&ptr, "=");
        char *value = strsep(&ptr, "&");
        if (key && value) {
            int val = atoi(value);
            if (strcmp(key, "r") == 0) {
                s_color.r = (val >= 0 && val <= 255) ? val : s_color.r;
            } else if (strcmp(key, "g") == 0) {
                s_color.g = (val >= 0 && val <= 255) ? val : s_color.g;
            } else if (strcmp(key, "b") == 0) {
                s_color.b = (val >= 0 && val <= 255) ? val : s_color.b;
            }
            if (s_led_state) {
                /* Set the LED pixel using RGB from 0 (0%) to 255 (100%) for each color */
                led_strip_set_pixel(led_strip, 0, s_color.g, s_color.r, s_color.b);
                /* Refresh the strip to send data */
                led_strip_refresh(led_strip);
            }
            
        }
    }

    // if (s_led_state) {
    //     set_light_color(r, g, b);
    // }

    httpd_resp_set_status(req, "303 See Other");
    httpd_resp_set_hdr(req, "Location", "/light");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

void set_light_color(uint8_t r, uint8_t g, uint8_t b) {
    s_color.r = r;
    s_color.g = g;
    s_color.b = b;
}

static const httpd_uri_t light_control_get = {
    .uri       = "/light",
    .method    = HTTP_GET,
    .handler   = light_control_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t light_control_post = {
    .uri       = "/light",
    .method    = HTTP_POST,
    .handler   = light_control_handler,
    .user_ctx  = NULL
};

static const httpd_uri_t color_control_post = {
    .uri       = "/set_rgb",
    .method    = HTTP_POST,
    .handler   = color_control_handler,
    .user_ctx  = NULL
};
/* This handler allows the custom error handling functionality to be
 * tested from client side. For that, when a PUT request 0 is sent to
 * URI /ctrl, the /hello and /echo URIs are unregistered and following
 * custom error handler http_404_error_handler() is registered.
 * Afterwards, when /hello or /echo is requested, this custom error
 * handler is invoked which, after sending an error message to client,
 * either closes the underlying socket (when requested URI is /echo)
 * or keeps it open (when requested URI is /hello). This allows the
 * client to infer if the custom error handler is functioning as expected
 * by observing the socket state.
 */
esp_err_t http_404_error_handler(httpd_req_t *req, httpd_err_code_t err)
{
    if (strcmp("/hello", req->uri) == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/hello URI is not available");
        /* Return ESP_OK to keep underlying socket open */
        return ESP_OK;
    } else if (strcmp("/echo", req->uri) == 0) {
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "/echo URI is not available");
        /* Return ESP_FAIL to close underlying socket */
        return ESP_FAIL;
    }
    /* For any other URI send 404 and close socket */
    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "Some 404 error message");
    return ESP_FAIL;
}

static httpd_handle_t start_webserver(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    config.lru_purge_enable = true;

    // Start the httpd server
    ESP_LOGI(TAG, "Starting server on port: '%d'", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        // Set URI handlers
        ESP_LOGI(TAG, "Registering URI handlers");
        httpd_register_uri_handler(server, &hello);
        httpd_register_uri_handler(server, &root);
        httpd_register_uri_handler(server, &light_control_get);
        httpd_register_uri_handler(server, &light_control_post);
        httpd_register_uri_handler(server, &color_control_post);
        return server;
    }

    ESP_LOGI(TAG, "Error starting server!");
    return NULL;
}

#if !CONFIG_IDF_TARGET_LINUX
static esp_err_t stop_webserver(httpd_handle_t server)
{
    // Stop the httpd server
    return httpd_stop(server);
}

static void disconnect_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server) {
        ESP_LOGI(TAG, "Stopping webserver");
        if (stop_webserver(*server) == ESP_OK) {
            *server = NULL;
        } else {
            ESP_LOGE(TAG, "Failed to stop http server");
        }
    }
}

static void connect_handler(void* arg, esp_event_base_t event_base,
                            int32_t event_id, void* event_data)
{
    httpd_handle_t* server = (httpd_handle_t*) arg;
    if (*server == NULL) {
        ESP_LOGI(TAG, "Starting webserver");
        *server = start_webserver();
    }
}
#endif // !CONFIG_IDF_TARGET_LINUX

esp_err_t init_fs(void)
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/www",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = false
    };
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return ESP_FAIL;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }
    return ESP_OK;
}

void app_main(void)
{
    static httpd_handle_t server = NULL;

    configure_led();
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());
    ESP_ERROR_CHECK(init_fs());
    /* Register event handlers to stop the server when Wi-Fi or Ethernet is disconnected,
     * and re-start it upon connection.
     */

    /* Start the server for the first time */
    server = start_webserver();

    while (server) {
        sleep(5);
    }
}
