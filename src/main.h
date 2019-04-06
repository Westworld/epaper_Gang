
void myDelay(long delay);

void showImage();
void showBitmapFrom_HTTP(const char* host, const char* path, const char* filename, int16_t x, int16_t y, bool with_color);
uint32_t read32(WiFiClient& client);
uint16_t read16(WiFiClient& client);
uint32_t skip(WiFiClient& client, int32_t bytes);
uint32_t read(WiFiClient& client, uint8_t* buffer, int32_t bytes);
void helloWorld(char *HelloWorld);
void ReportError(char *errormessage);