
#ifndef __HTTP_HEADER__
#define __HTTP_HEADER__

#define MAX_HTTP_RECV_BUFFER 512
#define MAX_HTTP_OUTPUT_BUFFER 512

struct url_data {
    const char *token;
    const char *key;
    const char *query;
    int value;
    //char resp[40];
};

void init_http(void);
void http_post(struct url_data *data);
//void http_get(struct url_data *data);

#endif // __HTTP_HEADER__

