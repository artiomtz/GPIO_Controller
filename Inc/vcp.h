void powerUpInit(void); // check connection

void receiveData(void); // Rx
void processData(void); // process
void transmitData(void); // Tx

void checkErrors(void); // display error

void adcCalibration(ADC_HandleTypeDef *hadc); // ADC calibration
void checkMeasurement(ADC_HandleTypeDef *hadc, int adcNum); // check +15v and -15v input

void sleepCheck(void); // check sleep timer
_Bool powerOutputEnableCheck(void); // check PE14
void powerIsOnSet(_Bool set); // set PE14

void loadFromFlash (void); // load data from flash at power up
