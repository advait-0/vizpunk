#include <esp_http_server.h>
#include "handlers.h"

// Serve landing page
esp_err_t page_handler(httpd_req_t *req) {
    const char *html = "<!DOCTYPE html>"
        "<html lang='en'>"
        "<head>"
        "<meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
        "<title>ESP32 Control Panel</title>"
        "<style>"
        "body { font-family: Arial, sans-serif; display: flex; justify-content: center; align-items: center; height: 100vh; background-color: #f4f4f4; margin: 0; }"
        ".container { text-align: center; background: white; padding: 20px; border-radius: 10px; box-shadow: 0 4px 10px rgba(0,0,0,0.1); width: 90%; max-width: 400px; }"
        "h1 { margin-bottom: 20px; }"
        ".btn { display: block; width: 100%; padding: 12px; margin: 10px 0; font-size: 16px; border: none; border-radius: 5px; cursor: pointer; }"
        ".btn.adc { background: #007BFF; color: white; }"
        ".btn.function { background: #28A745; color: white; }"
        ".btn.psu { background: #FFC107; color: white; }"
        ".btn:hover { opacity: 0.9; }"
        "</style>"
        "</head>"
        "<body>"
        "<div class='container'>"
        "<h1>ESP32 Control Panel</h1>"
        "<button class='btn adc' onclick=\"location.href='/adc'\">ADC</button>"
        "<button class='btn function' onclick=\"location.href='/function_generator'\">Function Generator</button>"
        "<button class='btn psu' onclick=\"location.href='/psu'\">PSU</button>"
        "</div>"
        "</body>"
        "</html>";

    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Serve ADC page
esp_err_t adc_handler(httpd_req_t *req) {
    const char *html = "<!DOCTYPE html>"
        "<html lang='en'><head><meta charset='UTF-8'>"
        "<meta name='viewport' content='width=device-width, initial-scale=1.0'>"
        "<title>ESP32 Voltage Graph</title>"
        "<script src='/uPlot.iife.min.js'></script>"
        "<style>"
        "body { font-family: Arial, sans-serif; text-align: center; padding: 20px; }"
        ".container { max-width: 600px; margin: auto; padding: 20px; border: 2px solid #ddd; border-radius: 10px; box-shadow: 2px 2px 12px #aaa; }"
        "#chart { width: 100%; height: 300px; margin-top: 20px; }"
        "</style></head>"
        "<body>"
        "<h1>ESP32 Voltage Monitoring</h1>"
        "<div class='container'>"
        "<button onclick='location.reload()'>Refresh Graph</button>"
        "</div>"
        "<h2>Live Voltage Graph</h2><div id='chart'></div>"
        "<script>"
        "let data = [[], []];"
        "let maxDataPoints = 50;"
        "const startTime = Date.now();"

        "const opts = {"
        "    title: 'Voltage vs. Time',"
        "    width: 600,"
        "    height: 300,"
        "    scales: { x: { time: false }, y: { min: 0, max: 5000 } },"
        "    axes: ["
        "        { label: 'Time (s)' },"
        "        { label: 'Voltage (mV)' }"
        "    ],"
        "    series: [ {}, { label: 'Voltage', stroke: 'red', width: 2 } ]"
        "};"

        "document.addEventListener('DOMContentLoaded', () => {"
        "    if (typeof uPlot !== 'undefined') {"
        "        window.chart = new uPlot(opts, data, document.getElementById('chart'));"
        "        setInterval(fetchData, 1000);"
        "    } else { console.error('uPlot failed to load.'); }"
        "});"

        "function fetchData() {"
        "    fetch('/graph')"
        "    .then(res => res.text())"
        "    .then(value => {"
        "        let voltage = parseFloat(value);"
        "        let elapsedTime = ((Date.now() - startTime) / 1000).toFixed(2);"
        "        if (!isNaN(voltage)) {"
        "            data[0].push(parseFloat(elapsedTime));"
        "            data[1].push(voltage);"
        "            if (data[0].length > maxDataPoints) {"
        "                data[0].shift();"
        "                data[1].shift();"
        "            }"
        "            window.chart.setData(data);"
        "        }"
        "    })"
        "    .catch(err => console.error('Fetch error:', err));"
        "}"
        "</script></body></html>";
    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// HTTP handler for UI page
esp_err_t function_generator_handler(httpd_req_t *req) {
    const char *html = "<!DOCTYPE html>"
        "<html><head><title>Function Generator</title>"
        "<style>"
        "body {font-family: Arial, sans-serif; text-align: center; padding: 20px;}"
        "input, select, button {margin: 10px; padding: 10px; font-size: 16px;}"
        ".container {max-width: 400px; margin: auto; padding: 20px; border: 2px solid #ddd; border-radius: 10px; box-shadow: 2px 2px 12px #aaa;}"
        "canvas {border: 1px solid #ddd; width: 100%; max-width: 400px; height: 200px;}"
        "</style></head>"
        "<body><h1>Function Generator</h1>"
        "<div class='container'>"
        "<label>Frequency (Hz):</label><br>"
        "<input type='number' id='freq' min='1' max='100000' value='1000'><br>"
        "<label>Waveform Type:</label><br>"
        "<select id='waveform'>"
        "<option value='sine'>Sine</option>"
        "<option value='square'>Square</option>"
        "<option value='triangle'>Triangle</option>"
        "</select><br>"
        "<button onclick='sendData()'>Set Function Generator</button>"
        "<canvas id='graph'></canvas>"
        "</div>"
        "<script>"
        "function sendData() {"
        "   let freq = document.getElementById('freq').value;"
        "   let wave = document.getElementById('waveform').value;"
        "   fetch('/set_function_generator?freq=' + freq + '&wave=' + wave)"
        "       .then(response => response.text())"
        "       .then(data => alert(data));"
        "}"
        "function drawGraph() {"
        "   let canvas = document.getElementById('graph');"
        "   let ctx = canvas.getContext('2d');"
        "   ctx.clearRect(0, 0, canvas.width, canvas.height);"
        "   ctx.beginPath();"
        "   for (let i = 0; i < canvas.width; i++) {"
        "       let y = 100 + 50 * Math.sin(i * 0.05);"
        "       ctx.lineTo(i, y);"
        "   }"
        "   ctx.strokeStyle = 'blue';"
        "   ctx.lineWidth = 2;"
        "   ctx.stroke();"
        "}"
        "setInterval(drawGraph, 1000);"
        "</script>"
        "</body></html>";

    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t dummy_handler(httpd_req_t *req) {
    if (strcmp(req->uri, "/favicon.ico") == 0) {
        // Serve a blank favicon to prevent unnecessary requests in the logs
        httpd_resp_set_type(req, "image/x-icon");
        httpd_resp_send(req, "", HTTPD_RESP_USE_STRLEN);  // Empty response for favicon
    } else if (strcmp(req->uri, "/chat") == 0) {
        // Handle chat endpoint
        if (req->method == HTTP_GET) {
            httpd_resp_send(req, "Chat endpoint (GET)", HTTPD_RESP_USE_STRLEN);
        } else if (req->method == HTTP_POST) {
            httpd_resp_send(req, "Chat endpoint (POST)", HTTPD_RESP_USE_STRLEN);
        } else {
            httpd_resp_send_err(req, HTTPD_405_METHOD_NOT_ALLOWED, "Method Not Allowed");
        }
    } else {
        // Default empty response for unhandled URIs
        httpd_resp_send(req, "", HTTPD_RESP_USE_STRLEN);
    }
    return ESP_OK;
}

// HTTP handler for processing function generator settings
esp_err_t set_function_generator_handler(httpd_req_t *req) {
    char query[100], freq_str[10] = {0}, wave[10] = {0};
    
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        httpd_query_key_value(query, "freq", freq_str, sizeof(freq_str));
        httpd_query_key_value(query, "wave", wave, sizeof(wave));
    }

    int function_generator_freq = atoi(freq_str);
    ESP_LOGI(TAG, "Received: Frequency = %d Hz, Waveform = %s", function_generator_freq, wave);

    uint16_t freq_lsb, freq_msb;
    calculate_frequency_registers(function_generator_freq, &freq_lsb, &freq_msb);
    send_ad9833(freq_lsb, freq_msb, wave);

    httpd_resp_send(req, "Function generator settings updated!", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Serve PSU page
esp_err_t psu_handler(httpd_req_t *req) {
    const char *html = "<html><body><h1>PSU Settings</h1>"
        "<p>Set output voltage and current.</p>"
        "</body></html>";
    httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Serve real-time sensor data
esp_err_t graph_handler(httpd_req_t *req) {
    char data[16];
    snprintf(data, sizeof(data), "%d", get_sensor_value());
    httpd_resp_send(req, data, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Serve JavaScript file from SPIFFS
esp_err_t js_handler(httpd_req_t *req) {
    FILE *f = fopen("/spiffs/uPlot.iife.min.js", "r");  
    if (!f) {
        ESP_LOGE(LOG_TAG, "Failed to open uPlot.iife.min.js in SPIFFS");
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    char buffer[1024];
    size_t read_bytes;
    httpd_resp_set_type(req, "application/javascript");

    while ((read_bytes = fread(buffer, 1, sizeof(buffer), f)) > 0) {
        httpd_resp_send_chunk(req, buffer, read_bytes);
    }

    fclose(f);
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}