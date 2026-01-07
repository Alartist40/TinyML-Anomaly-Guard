idf_component_register(SRCS "main.cpp" "model.cpp" "capture.cpp" "train.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES nvs_flash driver esp_wifi esp_now)
