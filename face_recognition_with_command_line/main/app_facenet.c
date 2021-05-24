/* ESPRESSIF MIT License
 * 
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 * 
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include <string.h>
#include <math.h>
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "app_facenet.h"
#include "sdkconfig.h"

#include "driver/gpio.h"

#include "fr_flash.h"
#include "esp_partition.h"


#define ENROLL_BUTTON_PIN GPIO_NUM_14
#define RECOGNIZE_BUTTON_PIN GPIO_NUM_13
//gpio_get_level(gpio_num_t gpio_num);
//gpio_set_level(gpio_num_t gpio_num, uint32_t level);

static const char *TAG = "app_process";

//#define ENROLL_CONFIRM_TIMES 3
//#define FACE_ID_SAVE_NUMBER 1
static face_id_list id_list = {0};
char *number_suffix(int32_t number)
{
    uint8_t n = number % 10;

    if (n == 0)
        return "zero";
    else if (n == 1)
        return "st";
    else if (n == 2)
        return "nd";
    else if (n == 3)
        return "rd";
    else
        return "th";
}

mtmn_config_t init_config()
{
    mtmn_config_t mtmn_config = {0};
    mtmn_config.type = FAST;
    mtmn_config.min_face = 80;
    mtmn_config.pyramid = 0.707;
    mtmn_config.pyramid_times = 4;
    mtmn_config.p_threshold.score = 0.6;
    mtmn_config.p_threshold.nms = 0.7;
    mtmn_config.p_threshold.candidate_number = 20;
    mtmn_config.r_threshold.score = 0.7;
    mtmn_config.r_threshold.nms = 0.7;
    mtmn_config.r_threshold.candidate_number = 10;
    mtmn_config.o_threshold.score = 0.7;
    mtmn_config.o_threshold.nms = 0.7;
    mtmn_config.o_threshold.candidate_number = 1;

    return mtmn_config;
}

void task_process(void *arg)
{ /*{{{*/

    //*
    gpio_reset_pin(ENROLL_BUTTON_PIN);
    gpio_reset_pin(RECOGNIZE_BUTTON_PIN);
    gpio_set_pull_mode(ENROLL_BUTTON_PIN, GPIO_PULLUP_ENABLE);
    gpio_set_pull_mode(RECOGNIZE_BUTTON_PIN, GPIO_PULLUP_ENABLE);
    gpio_set_direction(ENROLL_BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(RECOGNIZE_BUTTON_PIN, GPIO_MODE_INPUT);
    //*/

    size_t frame_num = 0;
    dl_matrix3du_t *image_matrix = NULL;
    camera_fb_t *fb = NULL;

    /* 1. Load configuration for detection */
    mtmn_config_t mtmn_config = init_config();

    /* 2. Preallocate matrix to store aligned face 56x56  */
    dl_matrix3du_t *aligned_face = dl_matrix3du_alloc(1,
                                                      FACE_WIDTH,
                                                      FACE_HEIGHT,
                                                      3);

    // load id list from flash
    int id_count = read_face_id_from_flash(&id_list);
    ESP_LOGI(TAG, "\nRead %d id-s from flash\n", id_count);

    int8_t is_enrolling = 1;
    if (id_count == FACE_ID_SAVE_NUMBER)
    {
        is_enrolling = 0;
        ESP_LOGI(TAG, "\n>>> Max ID enrolled <<<\n");
    }
 
    int8_t count_down_second = 3; //second
    int32_t next_enroll_index = 0;
    int8_t left_sample_face;

    ESP_LOGI(TAG, "\nInit finished!\n");
    vTaskDelay(5000 / portTICK_PERIOD_MS);

    do
    {
        int64_t start_time = esp_timer_get_time();

        //*
        int enroll_button = gpio_get_level(ENROLL_BUTTON_PIN);
        int recognize_button = gpio_get_level(RECOGNIZE_BUTTON_PIN);

        // ******************************************************************************************************
        // ******************************************** ENROLL **************************************************
        // ******************************************************************************************************

        if(enroll_button == 0)
        {
            ESP_LOGI(TAG, "<<< ENROLL BUTTON >>>\n");

            if(is_enrolling)
            {

                // 3. Get one image with camera 
                fb = esp_camera_fb_get();
                if (!fb)
                {
                    ESP_LOGE(TAG, "Camera capture failed");
                    continue;
                }
                int64_t fb_get_time = esp_timer_get_time();
                //ESP_LOGI(TAG, "Get one frame in %lld ms.", (fb_get_time - start_time) / 1000);

                // 4. Allocate image matrix to store RGB data 
                image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);

                // 5. Transform image to RGB 
                uint32_t res = fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item);
                if (true != res)
                {
                    ESP_LOGE(TAG, "fmt2rgb888 failed, fb: %d", fb->len);
                    dl_matrix3du_free(image_matrix);
                    continue;
                }

                esp_camera_fb_return(fb);

                // 6. Do face detection 
                box_array_t *net_boxes = face_detect(image_matrix, &mtmn_config);
                //ESP_LOGI(TAG, "Detection time consumption: %lldms", (esp_timer_get_time() - fb_get_time) / 1000);

                if (net_boxes)
                {
                    frame_num++;
                    ESP_LOGI(TAG, "Face Detection Count: %d", frame_num);

                    // 5. Do face alignment 
                    if (align_face(net_boxes, image_matrix, aligned_face) == ESP_OK)
                    {
                        // 6. Do face enrollment 
                        //left_sample_face = enroll_face(&id_list, aligned_face);
                        left_sample_face = enroll_face_id_to_flash(&id_list, aligned_face);
                        ESP_LOGI(TAG, "Face ID Enrollment: Take the %d%s sample",
                                ENROLL_CONFIRM_TIMES - left_sample_face,
                                number_suffix(ENROLL_CONFIRM_TIMES - left_sample_face));

                        if (left_sample_face == 0)
                        {
                            next_enroll_index++;
                            ESP_LOGI(TAG, "\nEnrolled Face ID: %d", id_list.tail);

                            if (id_list.count == FACE_ID_SAVE_NUMBER)
                            {
                                is_enrolling = 0;
                                ESP_LOGI(TAG, "\n>>> Max ID enrolled <<<\n");
                                vTaskDelay(5000 / portTICK_PERIOD_MS);
                            }
                            else
                            {
                                ESP_LOGI(TAG, "Please log in another one.");
                                vTaskDelay(5000 / portTICK_PERIOD_MS);
                            }
                        }
                    }
                    else
                    {
                        ESP_LOGI(TAG, "Detected face is not proper.");
                    }

                    dl_lib_free(net_boxes->score);
                    dl_lib_free(net_boxes->box);
                    dl_lib_free(net_boxes->landmark);
                    dl_lib_free(net_boxes);

                }//  if (net_boxes)

                dl_matrix3du_free(image_matrix);

            }//  if(is_enrolling)
            else
            {
                ESP_LOGI(TAG, "No place for new faces!");
                vTaskDelay(5000 / portTICK_PERIOD_MS);
            }

        }// if(enroll_button == 0)
        // ******************************************************************************************************
        // **************************************** END ENROLL **************************************************
        // ******************************************************************************************************


        // ******************************************************************************************************
        // ****************************************RECOGNITION **************************************************
        // ******************************************************************************************************
        if(recognize_button == 0)
        {
            ESP_LOGI(TAG, "<<< RECOGNIZE BUTTON >>>\n");

        	// 3. Get one image with camera 
        	fb = esp_camera_fb_get();
        	if (!fb)
        	{
            	ESP_LOGE(TAG, "Camera capture failed");
            	continue;
        	}
            int64_t fb_get_time = esp_timer_get_time();
            //ESP_LOGI(TAG, "Get one frame in %lld ms.", (fb_get_time - start_time) / 1000);

            // 4. Allocate image matrix to store RGB data 
            image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);

            // 5. Transform image to RGB 
            uint32_t res = fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item);
            if (true != res)
            {
                ESP_LOGE(TAG, "fmt2rgb888 failed, fb: %d", fb->len);
                dl_matrix3du_free(image_matrix);
                continue;
            }

            esp_camera_fb_return(fb);

            // 6. Do face detection 
            box_array_t *net_boxes = face_detect(image_matrix, &mtmn_config);
            //ESP_LOGI(TAG, "Detection time consumption: %lldms", (esp_timer_get_time() - fb_get_time) / 1000);

            if (net_boxes)
            {
                frame_num++;
                ESP_LOGI(TAG, "Face Detection Count: %d", frame_num);

                // 5. Do face alignment 
                if (align_face(net_boxes, image_matrix, aligned_face) == ESP_OK)
                {

                    // 6. Do face recognition 
                    int64_t recog_match_time = esp_timer_get_time();

                    int matched_id = recognize_face(&id_list, aligned_face);
                    if (matched_id >= 0)
                    {
                        ESP_LOGI(TAG, "Matched Face ID: %d", matched_id);
                        vTaskDelay(5000 / portTICK_PERIOD_MS);
                    }
                    else
                    {
                        ESP_LOGI(TAG, "No Matched Face ID");
                        vTaskDelay(5000 / portTICK_PERIOD_MS);
                    }

                    //ESP_LOGI(TAG, "Recognition time consumption: %lldms\n",
                    //        (esp_timer_get_time() - recog_match_time) / 1000);
                }
                else
                {
                    ESP_LOGI(TAG, "Detected face is not proper.");
                }

                dl_lib_free(net_boxes->score);
                dl_lib_free(net_boxes->box);
                dl_lib_free(net_boxes->landmark);
                dl_lib_free(net_boxes);

            }//  if (net_boxes)

            dl_matrix3du_free(image_matrix);

        }
        // ******************************************************************************************************
        // ************************************END RECOGNITION **************************************************
        // ******************************************************************************************************


        // */


        /*

        // 3. Get one image with camera 
        fb = esp_camera_fb_get();
        if (!fb)
        {
            ESP_LOGE(TAG, "Camera capture failed");
            continue;
        }
        int64_t fb_get_time = esp_timer_get_time();
        ESP_LOGI(TAG, "Get one frame in %lld ms.", (fb_get_time - start_time) / 1000);

        // 4. Allocate image matrix to store RGB data 
        image_matrix = dl_matrix3du_alloc(1, fb->width, fb->height, 3);

        // 5. Transform image to RGB 
        uint32_t res = fmt2rgb888(fb->buf, fb->len, fb->format, image_matrix->item);
        if (true != res)
        {
            ESP_LOGE(TAG, "fmt2rgb888 failed, fb: %d", fb->len);
            dl_matrix3du_free(image_matrix);
            continue;
        }

        esp_camera_fb_return(fb);

        // 6. Do face detection 
        box_array_t *net_boxes = face_detect(image_matrix, &mtmn_config);
        ESP_LOGI(TAG, "Detection time consumption: %lldms", (esp_timer_get_time() - fb_get_time) / 1000);

        if (net_boxes)
        {
            frame_num++;
            ESP_LOGI(TAG, "Face Detection Count: %d", frame_num);

            // 5. Do face alignment 
            if (align_face(net_boxes, image_matrix, aligned_face) == ESP_OK)
            {
                //count down
                while (count_down_second > 0)
                {
                    ESP_LOGI(TAG, "Face ID Enrollment Starts in %ds.\n", count_down_second);

                    vTaskDelay(1000 / portTICK_PERIOD_MS);

                    count_down_second--;

                    if (count_down_second == 0)
                        ESP_LOGI(TAG, "\n>>> Face ID Enrollment Starts <<<\n");
                }

                // 6. Do face enrollment 
                if (is_enrolling == 1)
                {
                    left_sample_face = enroll_face(&id_list, aligned_face);
                    ESP_LOGI(TAG, "Face ID Enrollment: Take the %d%s sample",
                             ENROLL_CONFIRM_TIMES - left_sample_face,
                             number_suffix(ENROLL_CONFIRM_TIMES - left_sample_face));

                    if (left_sample_face == 0)
                    {
                        next_enroll_index++;
                        ESP_LOGI(TAG, "\nEnrolled Face ID: %d", id_list.tail);

                        if (id_list.count == FACE_ID_SAVE_NUMBER)
                        {
                            is_enrolling = 0;
                            ESP_LOGI(TAG, "\n>>> Face Recognition Starts <<<\n");
                            vTaskDelay(2000 / portTICK_PERIOD_MS);
                        }
                        else
                        {
                            ESP_LOGI(TAG, "Please log in another one.");
                            vTaskDelay(2500 / portTICK_PERIOD_MS);
                        }
                    }
                }
                // 6. Do face recognition 
                else
                {
                    int64_t recog_match_time = esp_timer_get_time();

                    int matched_id = recognize_face(&id_list, aligned_face);
                    if (matched_id >= 0)
                        ESP_LOGI(TAG, "Matched Face ID: %d", matched_id);
                    else
                        ESP_LOGI(TAG, "No Matched Face ID");

                    ESP_LOGI(TAG, "Recognition time consumption: %lldms\n",
                             (esp_timer_get_time() - recog_match_time) / 1000);
                }
            }
            else
            {
                ESP_LOGI(TAG, "Detected face is not proper.");
            }

            dl_lib_free(net_boxes->score);
            dl_lib_free(net_boxes->box);
            dl_lib_free(net_boxes->landmark);
            dl_lib_free(net_boxes);
        }

        dl_matrix3du_free(image_matrix);

        //*/

        //vTaskDelay(2000 / portTICK_PERIOD_MS);

    } while (1);
} /*}}}*/

void app_facenet_main()
{
    face_id_init(&id_list, FACE_ID_SAVE_NUMBER, ENROLL_CONFIRM_TIMES);
    xTaskCreatePinnedToCore(task_process, "process", 4 * 1024, NULL, 5, NULL, 1);
}
