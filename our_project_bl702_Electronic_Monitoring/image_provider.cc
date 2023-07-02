#include "image_provider.h"
#include "misc.h"
#include "tensorflow/lite/micro/examples/person_detection/model_settings.h"
//#include "tensorflow/lite/micro/examples/person_detection/testdata/person_image_data.h"
//#include "tensorflow/lite/micro/examples/person_detection/testdata/no_person_image_data.h"
TfLiteStatus GetImage(tflite::ErrorReporter* error_reporter, int image_width,
                      int image_height, int channels, int8_t* image_data, int8_t* scale_img) {

  for (int i = 0; i < image_width * image_height * channels; ++i) {
    image_data[i] = scale_img[i];
  	}
  return kTfLiteOk;
}
