#include <microhttpd.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <postgresql/libpq-fe.h>
#include "cJSON.h"

#define PORT 8888

const char *conninfo = "host=localhost port=5432 dbname=postgres_db user=user password=password";

void create_table_if_not_exists(PGconn *conn) {
    const char *create_table_query =
        "CREATE TABLE IF NOT EXISTS adc_logs ("
        "id SERIAL PRIMARY KEY,"
        "voltage_adc1 REAL NOT NULL,"
        "voltage_adc2 REAL NOT NULL,"
        "timestamp TIMESTAMP DEFAULT CURRENT_TIMESTAMP"
        ");";

    PGresult *res = PQexec(conn, create_table_query);
    if (PQresultStatus(res) != PGRES_COMMAND_OK) {
        fprintf(stderr, "Failed to create table: %s", PQerrorMessage(conn));
    }
    PQclear(res);
}

int handle_post_request(const char *data, size_t size) {
    cJSON *json = cJSON_ParseWithLength(data, size);
    if (!json) {
        fprintf(stderr, "Error parsing JSON\n");
        return 0;
    }

    cJSON *voltage_adc1_item = cJSON_GetObjectItemCaseSensitive(json, "voltage_adc1");
    cJSON *voltage_adc2_item = cJSON_GetObjectItemCaseSensitive(json, "voltage_adc2");

    if (!cJSON_IsNumber(voltage_adc1_item) || !cJSON_IsNumber(voltage_adc2_item)) {
        fprintf(stderr, "Invalid JSON format\n");
        cJSON_Delete(json);
        return 0;
    }

    float voltage_adc1 = voltage_adc1_item->valuedouble;
    float voltage_adc2 = voltage_adc2_item->valuedouble;

    cJSON_Delete(json);

    PGconn *conn = PQconnectdb(conninfo);
    if (PQstatus(conn) != CONNECTION_OK) {
        fprintf(stderr, "Connection to database failed: %s", PQerrorMessage(conn));
        PQfinish(conn);
        return 0;
    }

    create_table_if_not_exists(conn);

    const char *paramValues[2];
    char adc1_str[16], adc2_str[16];
    snprintf(adc1_str, sizeof(adc1_str), "%f", voltage_adc1);
    snprintf(adc2_str, sizeof(adc2_str), "%f", voltage_adc2);
    paramValues[0] = adc1_str;
    paramValues[1] = adc2_str;

    PGresult *res = PQexecParams(conn,
                                 "INSERT INTO adc_logs (voltage_adc1, voltage_adc2) VALUES ($1, $2);",
                                 2,
                                 NULL,
                                 paramValues,
                                 NULL,
                                 NULL,
                                 0);

    if (PQresultStatus(res) != PGRES_COMMAND_OK) {
        fprintf(stderr, "INSERT failed: %s", PQerrorMessage(conn));
        PQclear(res);
        PQfinish(conn);
        return 0;
    }

    PQclear(res);
    PQfinish(conn);
    return 1;
}

static enum MHD_Result request_handler(void *cls, struct MHD_Connection *connection,
                                      const char *url, const char *method,
                                      const char *version, const char *upload_data,
                                      size_t *upload_data_size, void **con_cls)
{
    static char *post_data = NULL;
    static size_t total_size = 0;

    if (strcmp(method, "POST") == 0 && strcmp(url, "/log_adc") == 0) {
        if (*upload_data_size != 0) {
            post_data = realloc(post_data, total_size + *upload_data_size + 1);
            if (post_data == NULL) {
                fprintf(stderr, "Out of memory\n");
                return MHD_NO;
            }
            memcpy(post_data + total_size, upload_data, *upload_data_size);
            total_size += *upload_data_size;
            post_data[total_size] = '\0';
            *upload_data_size = 0;
            return MHD_YES;
        } else {
            int success = handle_post_request(post_data, total_size);
            free(post_data);
            post_data = NULL;
            total_size = 0;

            const char *response_text = success ? "Data received and logged." : "Failed to process data.";
            struct MHD_Response *response = MHD_create_response_from_buffer(strlen(response_text),
                                                                            (void *)response_text,
                                                                            MHD_RESPMEM_PERSISTENT);
            enum MHD_Result ret = MHD_queue_response(connection, success ? MHD_HTTP_OK : MHD_HTTP_BAD_REQUEST, response);
            MHD_destroy_response(response);
            return ret;
        }
    }
    else if (strcmp(method, "GET") == 0 && strcmp(url, "/") == 0) {
        const char *page = "ESP32 Sensor Logger Server";
        struct MHD_Response *response = MHD_create_response_from_buffer(strlen(page),
                                                                        (void *)page,
                                                                        MHD_RESPMEM_PERSISTENT);
        enum MHD_Result ret = MHD_queue_response(connection, MHD_HTTP_OK, response);
        MHD_destroy_response(response);
        return ret;
    }
    else {
        const char *not_found = "Not Found";
        struct MHD_Response *response = MHD_create_response_from_buffer(strlen(not_found),
                                                                        (void *)not_found,
                                                                        MHD_RESPMEM_PERSISTENT);
        enum MHD_Result ret = MHD_queue_response(connection, MHD_HTTP_NOT_FOUND, response);
        MHD_destroy_response(response);
        return ret;
    }
}

int main()
{
    struct MHD_Daemon *daemon;

    daemon = MHD_start_daemon(MHD_USE_SELECT_INTERNALLY, PORT, NULL, NULL,
                              &request_handler, NULL, MHD_OPTION_END);
    if (NULL == daemon) {
        fprintf(stderr, "Failed to start HTTP server\n");
        return 1;
    }

    printf("HTTP server is running on port %d\n", PORT);
    printf("Press Enter to stop the server.\n");
    getchar();

    MHD_stop_daemon(daemon);
    return 0;
}
