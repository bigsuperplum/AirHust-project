#include <template.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/String.h>
#include <curl/curl.h>
#include <json/json.h>

// Base64编码函数
static std::string base64_encode(const unsigned char* data, size_t len) {
    const std::string base64_chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789+/";
    std::string ret;
    int i = 0;
    unsigned char char_array_3[3];
    unsigned char char_array_4[4];
    while (len--) {
        char_array_3[i++] = *(data++);
        if (i == 3) {
            char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
            char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
            char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
            char_array_4[3] = char_array_3[2] & 0x3f;
            for (i = 0; i < 4; i++) ret += base64_chars[char_array_4[i]];
            i = 0;
        }
    }
    if (i) {
        for (int j = i; j < 3; j++) char_array_3[j] = '\0';
        char_array_4[0] = (char_array_3[0] & 0xfc) >> 2;
        char_array_4[1] = ((char_array_3[0] & 0x03) << 4) + ((char_array_3[1] & 0xf0) >> 4);
        char_array_4[2] = ((char_array_3[1] & 0x0f) << 2) + ((char_array_3[2] & 0xc0) >> 6);
        char_array_4[3] = char_array_3[2] & 0x3f;
        for (int j = 0; j < i + 1; j++) ret += base64_chars[char_array_4[j]];
        while ((i++ < 3)) ret += '=';
    }
    return ret;
}

// 云端识别回调函数
size_t WriteCallback(void* contents, size_t size, size_t nmemb, std::string* response) {
    size_t totalSize = size * nmemb;
    response->append((char*)contents, totalSize);
    return totalSize;
}

// === 保留你原有的全局变量 ===
bool should_hover = false;
ros::Time hover_start_time;
float hover_duration = 12.0;
float hover_x, hover_y, hover_z, hover_yaw;
int mission_num = 0;
float if_debug = 0;
float err_max = 0.2;
ros::Publisher mavros_setpoint_pos_pub;
ros::Rate* global_rate = nullptr;

// === 新增：视觉相关变量 ===
ros::Publisher target_pub;
ros::Subscriber image_sub;
bool image_processed = false;
cv::Mat current_image;
bool image_received = false;

// === 新增：在全局变量部分添加 ===
std::string landmark_color;  // 存储起飞地标颜色
std::string landing_colors[3];  // 存储三个降落地标颜色
int color_compare_result = -1;  // 颜色比较结果：-1=未比较，0/1/2=匹配的降落地标索引
bool color_recognized = false;  // 起飞颜色是否已识别

// 云端识别相关变量
std::string access_token = "sk-786a18adf223459b8b6ff82dd57e7bcb"; // 需要替换为实际的token
std::string qwen_api_url = "https://dashscope.aliyuncs.com/api/v1/services/aigc/multimodal-generation/generation";

// === 新增：地标颜色识别函数 ===
std::string recognizeLandmarkColor(cv::Mat image)
{
    // 1. 编码图像
    std::vector<uchar> buffer;
    bool success = cv::imencode(".jpg", image, buffer);
    if (!success) {
        ROS_ERROR("Failed to encode image for color recognition.");
        return "";
    }
    std::string base64_image = base64_encode(buffer.data(), buffer.size());

    // 2. 构建颜色识别的提示词
    std::string prompt = "请识别图片中的地标颜色（单个主要颜色），只返回颜色的英文名称，比如'red'、'blue'、'green'等，不要其他文字。";

    // 3. 构建 JSON 请求体
    Json::Value root;
    root["model"] = "qwen3-vl-plus";
    Json::Value message;
    message["role"] = "user";
    Json::Value content_array;
    Json::Value image_obj;
    image_obj["image"] = "data:image/jpeg;base64," + base64_image;
    content_array.append(image_obj);
    Json::Value text_obj;
    text_obj["text"] = prompt;
    content_array.append(text_obj);
    message["content"] = content_array;
    root["input"]["messages"].append(message);
    root["parameters"]["temperature"] = 0.1;

    Json::StreamWriterBuilder writer;
    std::string request_body = Json::writeString(writer, root);

    // 4. 发送 HTTP 请求
    CURL* curl = curl_easy_init();
    std::string response_data;
    long http_code = 0;
    if (curl) {
        struct curl_slist* headers = nullptr;
        headers = curl_slist_append(headers, ("Authorization: Bearer " + access_token).c_str());
        headers = curl_slist_append(headers, "Content-Type: application/json");
        headers = curl_slist_append(headers, "X-DashScope-SSE: disable");

        curl_easy_setopt(curl, CURLOPT_URL, qwen_api_url.c_str());
        curl_easy_setopt(curl, CURLOPT_POST, 1L);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, request_body.c_str());
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_data);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30L);

        CURLcode res = curl_easy_perform(curl);
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code);

        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);

        // 调试信息
        ROS_INFO("=== Landmark Color Recognition ===");
        ROS_INFO("HTTP Status Code: %ld", http_code);

        if (res != CURLE_OK) {
            ROS_ERROR("CURL request failed: %s", curl_easy_strerror(res));
            return "";
        }
        if (http_code != 200) {
            ROS_ERROR("HTTP request failed with status code: %ld", http_code);
            return "";
        }

        // 5. 解析结果
        Json::Value response_root;
        Json::CharReaderBuilder reader;
        JSONCPP_STRING parse_err;
        std::istringstream iss(response_data);
        if (!Json::parseFromStream(reader, iss, &response_root, &parse_err)) {
            ROS_ERROR("JSON parsing failed: %s", parse_err.c_str());
            return "";
        }

        // 6. 提取颜色结果
        try {
            Json::Value content_array = response_root["output"]["choices"][0]["message"]["content"];
            if (!content_array.isArray() || content_array.size() == 0) {
                ROS_ERROR("Invalid 'content' field in response.");
                return "";
            }
            std::string result = content_array[0]["text"].asString();
            ROS_INFO("Extracted color result: [%s]", result.c_str());

            // 清理结果
            result.erase(0, result.find_first_not_of(" \t\n\r"));
            result.erase(result.find_last_not_of(" \t\n\r") + 1);
            ROS_INFO("Final cleaned color: [%s]", result.c_str());
            return result;
        }
        catch (const std::exception& e) {
            ROS_ERROR("Exception during color extraction: %s", e.what());
            return "";
        }
    }
    return "";
}

// === 新增：比较颜色函数 ===
int compareLandmarkColors() {
    if (landmark_color.empty()) {
        ROS_WARN("起飞地标颜色未识别，无法比较");
        return -1;
    }

    ROS_INFO("=== 开始比较地标颜色 ===");
    ROS_INFO("起飞地标颜色: %s", landmark_color.c_str());
    for (int i = 0; i < 3; i++) {
        ROS_INFO("降落地标%d颜色: %s", i, landing_colors[i].c_str());
        if (landing_colors[i] == landmark_color) {
            ROS_INFO("找到匹配的降落地标: 索引 %d", i);
            return i;
        }
    }

    ROS_WARN("未找到颜色匹配的降落地标");
    return -1;  // 没有匹配
}

// 云端图像识别函数
std::string recognizeWithQwen(cv::Mat image, int task_type) {
    // 1. 编码图像
    std::vector<uchar> buffer;
    bool success = cv::imencode(".jpg", image, buffer);
    if (!success) {
        ROS_ERROR("Failed to encode image to JPG.");
        return "";
    }
    std::string base64_image = base64_encode(buffer.data(), buffer.size());

    // 2. 构建提示词
    std::string prompt;
    switch (task_type) {
    case 9: prompt = "请识别图片中的字母，只返回识别到的字母内容，不要其他文字。"; break;
    case 10: prompt = "请扫描图片中的二维码，二维码内容应该是用逗号分隔的三个英文单词。只返回扫描到的二维码内容，不要其他文字。"; break;
    case 11: prompt = "请审慎地识别图片中的数字，数字不是4，6，7，只返回识别到的数字内容，不要其他文字。"; break;
    default: prompt = "请识别图片中的内容，只返回识别到的内容，不要其他文字。";
    }

    // 3. 构建 JSON 请求体
    Json::Value root;
    root["model"] = "qwen3-vl-plus";
    Json::Value message;
    message["role"] = "user";
    Json::Value content_array;
    Json::Value image_obj;
    image_obj["image"] = "data:image/jpeg;base64," + base64_image;
    content_array.append(image_obj);
    Json::Value text_obj;
    text_obj["text"] = prompt;
    content_array.append(text_obj);
    message["content"] = content_array;
    root["input"]["messages"].append(message);
    root["parameters"]["temperature"] = 0.1;

    Json::StreamWriterBuilder writer;
    std::string request_body = Json::writeString(writer, root);

    // === 开始发送 HTTP 请求 ===
    CURL* curl = curl_easy_init();
    std::string response_data;
    long http_code = 0;
    if (curl) {
        struct curl_slist* headers = nullptr;
        headers = curl_slist_append(headers, ("Authorization: Bearer " + access_token).c_str());
        headers = curl_slist_append(headers, "Content-Type: application/json");
        headers = curl_slist_append(headers, "X-DashScope-SSE: disable");

        curl_easy_setopt(curl, CURLOPT_URL, qwen_api_url.c_str());
        curl_easy_setopt(curl, CURLOPT_POST, 1L);
        curl_easy_setopt(curl, CURLOPT_POSTFIELDS, request_body.c_str());
        curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);
        curl_easy_setopt(curl, CURLOPT_WRITEFUNCTION, WriteCallback);
        curl_easy_setopt(curl, CURLOPT_WRITEDATA, &response_data);
        curl_easy_setopt(curl, CURLOPT_TIMEOUT, 30L);

        CURLcode res = curl_easy_perform(curl);
        curl_easy_getinfo(curl, CURLINFO_RESPONSE_CODE, &http_code); // 获取 HTTP 状态码

        curl_slist_free_all(headers);
        curl_easy_cleanup(curl);

        // === 打印关键调试信息 ===
        ROS_INFO("=== Qwen API Request Debug Info ===");
        ROS_INFO("HTTP Status Code: %ld", http_code);
        ROS_INFO("Request Body (first 200 chars): %s", request_body.substr(0, 200).c_str());
        ROS_INFO("Raw Response (first 500 chars): %s", response_data.substr(0, 500).c_str());

        if (res != CURLE_OK) {
            ROS_ERROR("CURL request failed: %s", curl_easy_strerror(res));
            return "";
        }
        if (http_code != 200) {
            ROS_ERROR("HTTP request failed with status code: %ld", http_code);
            ROS_ERROR("Full response body:\n%s", response_data.c_str());
            return "";
        }

        // === 尝试解析 JSON ===
        Json::Value response_root;
        Json::CharReaderBuilder reader;
        JSONCPP_STRING parse_err;
        std::istringstream iss(response_data);
        if (!Json::parseFromStream(reader, iss, &response_root, &parse_err)) {
            ROS_ERROR("JSON parsing failed: %s", parse_err.c_str());
            ROS_ERROR("Invalid JSON response:\n%s", response_data.c_str());
            return "";
        }

        // === 提取结果 ===
        try {
            Json::Value content_array = response_root["output"]["choices"][0]["message"]["content"];
            if (!content_array.isArray() || content_array.size() == 0) {
                ROS_ERROR("Invalid 'content' field in response: expected non-empty array.");
                ROS_ERROR("Response was:\n%s", response_data.c_str());
                return "";
            }
            std::string result = content_array[0]["text"].asString();
            ROS_INFO("Extracted raw result: [%s]", result.c_str());

            // 清理首尾空白（通常不需要清理冒号，因为模型不会加）
            result.erase(0, result.find_first_not_of(" \t\n\r"));
            result.erase(result.find_last_not_of(" \t\n\r") + 1);
            ROS_INFO("Final cleaned result: [%s]", result.c_str());
            return result;
        }
        catch (const std::exception& e) {
            ROS_ERROR("Exception during result extraction: %s", e.what());
            ROS_ERROR("Response structure was:\n%s", response_data.c_str());
            return "";
        }
    } // if(curl) 结束
    return ""; // curl 初始化失败时返回空
} // recognizeWithQwen 函数结束

void print_param() {
    std::cout << "=== 控制参数 ===" << std::endl;
    std::cout << "err_max: " << err_max << std::endl;
    std::cout << "ALTITUDE: " << ALTITUDE << std::endl;
    std::cout << "if_debug: " << if_debug << std::endl;
    if (if_debug == 1) std::cout << "自动offboard" << std::endl;
    else std::cout << "遥控器offboard" << std::endl;
}

// === 图像回调函数（使用通义千问云端识别）===
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    // 修改条件：增加对mission_num==1和12、13、14的判断（降落地标颜色识别）
    if (!should_hover || image_processed ||
        (mission_num != 1 && (mission_num < 9 || mission_num > 14))) {
        return;
    }

    ROS_INFO("=== imageCallback被触发 ===");
    ROS_INFO("should_hover: %d", should_hover);
    ROS_INFO("image_processed: %d", image_processed);
    ROS_INFO("mission_num: %d", mission_num);

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    current_image = cv_ptr->image.clone();
    image_received = true;

    // 在悬停期间，如果图像已接收且未处理，则进行识别
    if (image_received && !image_processed) {
        ROS_INFO("Processing image with Qwen API for mission %d", mission_num);

        std::string result;
        std::string location = "";  // 初始化为空字符串

        if (mission_num == 1) {
            // 任务1：识别起飞地标颜色
            result = recognizeLandmarkColor(current_image);
            if (!result.empty()) {
                landmark_color = result;  // 存储颜色值
                color_recognized = true;  // 标记已识别
                location = "0.0, 0.0";  // 起飞位置

                // 发布到/target话题，包含位置信息
                std_msgs::String msg_out;
                msg_out.data = "color:" + result + ",Location:[" + location + "]";
                target_pub.publish(msg_out);

                ROS_INFO("=========================================");
                ROS_INFO("发布起飞颜色识别结果");
                ROS_INFO("颜色: %s", result.c_str());
                ROS_INFO("位置: [%s]", location.c_str());
                ROS_INFO("话题: /target");
                ROS_INFO("=========================================");
            }
        }
        else if (mission_num == 12 || mission_num == 13 || mission_num == 14) {
            // 任务12、13、14：识别降落地标颜色
            result = recognizeLandmarkColor(current_image);
            if (!result.empty()) {
                int index = mission_num - 12;  // 12->0, 13->1, 14->2
                landing_colors[index] = result;

                // 设置对应位置
                if (mission_num == 12) {
                    location = "35.0, 3.0";
                }
                else if (mission_num == 13) {
                    location = "35.0, 1.0";
                }
                else {
                    location = "35.0, -1.0";
                }

                // 发布到/target话题，包含位置信息
                std_msgs::String msg_out;
                msg_out.data = "降落" + std::to_string(index) + "颜色:" + result + ",位置:[" + location + "]";
                target_pub.publish(msg_out);

                ROS_INFO("=========================================");
                ROS_INFO("发布降落地标颜色识别结果");
                ROS_INFO("索引: %d", index);
                ROS_INFO("颜色: %s", result.c_str());
                ROS_INFO("位置: [%s]", location.c_str());
                ROS_INFO("话题: /target");
                ROS_INFO("=========================================");
            }
        }
        else if (mission_num == 9 || mission_num == 10 || mission_num == 11) {
            // 任务9-11（字母、二维码、数字识别）
            result = recognizeWithQwen(current_image, mission_num);
            if (!result.empty()) {
                std_msgs::String msg_out;

                // 根据任务类型设置位置和标签
                std::string task_type;
                if (mission_num == 9) {
                    task_type = "letter";
                    location = "28.0, -1.0";
                }
                else if (mission_num == 10) {
                    task_type = "QR_code";
                    location = "28.0, 1.0";
                }
                else if (mission_num == 11) {
                    task_type = "number";
                    location = "28.0, 3.0";
                }

                // 发布到/target话题，包含位置信息
                msg_out.data = task_type + ":" + result + ",Locate:[" + location + "]";
                target_pub.publish(msg_out);

                ROS_INFO("=========================================");
                ROS_INFO("发布识别结果");
                ROS_INFO("任务类型: %s", task_type.c_str());
                ROS_INFO("识别结果: %s", result.c_str());
                ROS_INFO("位置坐标: [%s]", location.c_str());
                ROS_INFO("话题: /target");
                ROS_INFO("=========================================");
            }
        }

        if (!result.empty()) {
            image_processed = true; // 标记已处理
        }
        else {
            ROS_WARN("Recognition failed, no result returned");
        }
    }
}

// === 以下 main 函数完全保留你的原始逻辑（修改相关case）===
int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "template");
    ros::NodeHandle nh;
    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);
    ros::Subscriber local_pos_sub = nh.subscribe<nav_msgs::Odometry>("/mavros/local_position/odom", 10, local_pos_cb);

    // === 新增：视觉话题 ===
    target_pub = nh.advertise<std_msgs::String>("/target", 10);
    image_sub = nh.subscribe("/image_raw", 1, imageCallback);

    mavros_setpoint_pos_pub = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 100);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    ros::ServiceClient ctrl_pwm_client = nh.serviceClient<mavros_msgs::CommandLong>("mavros/cmd/command");

    ros::Rate rate(20);
    global_rate = &rate;

    nh.param<float>("err_max", err_max, 0.15);
    nh.param<float>("if_debug", if_debug, 1);
    print_param();

    int choice = 0;
    std::cout << "1 to go on , else to quit" << std::endl;
    std::cin >> choice;
    if (choice != 1) return 0;

    ros::spinOnce();
    rate.sleep();

    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    setpoint_raw.type_mask = /*1 + 2 + 4 + 8 + 16 + 32*/ +64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x = 0;
    setpoint_raw.position.y = 0;
    setpoint_raw.position.z = ALTITUDE;
    setpoint_raw.yaw = 0;

    for (int i = 100; ros::ok() && i > 0; --i) {
        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }

    std::cout << "ok" << std::endl;

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    ros::Time last_request = ros::Time::now();

    while (ros::ok()) {
        if (current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(3.0))) {
            if (if_debug == 1) {
                if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                    ROS_INFO("Offboard enabled");
                }
            }
            else {
                ROS_INFO("Waiting for OFFBOARD mode");
            }
            last_request = ros::Time::now();
        }
        else {
            if (!current_state.armed && (ros::Time::now() - last_request > ros::Duration(3.0))) {
                if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        if (fabs(local_pos.pose.pose.position.z - ALTITUDE) < 0.2) {
            if (ros::Time::now() - last_request > ros::Duration(0.5)) {
                mission_num = 1;
                last_request = ros::Time::now();
                break;
            }
        }

        mission_pos_cruise(0, 0, ALTITUDE, 0, err_max);
        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();
    }

    while (ros::ok()) {
        ROS_WARN("mission_num = %d", mission_num);
        switch (mission_num) {
        case 1:
            if (mission_pos_cruise(0, 0, ALTITUDE, 0, err_max))
            {
                // 到达起飞点，悬停进行颜色识别
                if (!should_hover)
                {
                    should_hover = true;
                    hover_start_time = ros::Time::now();
                    hover_x = local_pos.pose.pose.position.x;
                    hover_y = local_pos.pose.pose.position.y;
                    hover_z = local_pos.pose.pose.position.z;
                    hover_yaw = yaw;
                    image_processed = false;     // 重置图像处理标志
                    image_received = false;      // 重置图像接收标志
                    color_recognized = false;    // 重置颜色识别标志
                    ROS_INFO("开始悬停进行起飞地标颜色识别...");
                }

                // 悬停3秒进行颜色识别
                if ((ros::Time::now() - hover_start_time).toSec() >= 3.0)
                {
                    should_hover = false;
                    if (color_recognized)
                    {
                        ROS_INFO("起飞地标颜色识别完成: %s", landmark_color.c_str());
                    }
                    else
                    {
                        ROS_WARN("起飞地标颜色识别失败或超时");
                    }
                    mission_num = 2;  // 进入下一个任务
                    last_request = ros::Time::now();
                }
            }
            break;
        case 2:
            if (mission_pos_cruise(0.0, -3.0, ALTITUDE, 0.0, err_max)) {
                mission_num = 3;
                last_request = ros::Time::now();
            }
            break;
        case 3:
            if (mission_pos_cruise(5.0, -3.0, ALTITUDE, 0.0, err_max)) {
                mission_num = 4;
                last_request = ros::Time::now();
            }
            break;
        case 4:
            if (mission_pos_cruise(10.0, -3.0, ALTITUDE, 0.0, err_max)) {
                mission_num = 5;
                last_request = ros::Time::now();
            }
            break;
        case 5:
            if (mission_pos_cruise(15.0, -3.0, ALTITUDE, 0.0, err_max)) {
                mission_num = 6;
                last_request = ros::Time::now();
            }
            break;
        case 6:
            if (mission_pos_cruise(20.0, -3.0, ALTITUDE, 0.0, err_max)) {
                mission_num = 7;
                last_request = ros::Time::now();
            }
            break;
        case 7:
            if (mission_pos_cruise(28.0, -3.0, ALTITUDE, 0.0, err_max)) {
                mission_num = 8;
                last_request = ros::Time::now();
            }
            break;
        case 8:
            if (mission_pos_cruise(28.0, -1.0, ALTITUDE, 0.0, err_max)) {
                last_request = ros::Time::now();
                should_hover = true;
                hover_start_time = ros::Time::now();
                hover_x = local_pos.pose.pose.position.x;
                hover_y = local_pos.pose.pose.position.y;
                hover_z = local_pos.pose.pose.position.z;
                hover_yaw = yaw;
                image_processed = false;
                image_received = false;
                mission_num = 9;
            }
            break;
        case 9:  // 字母识别任务
            if (mission_pos_cruise(28.0, 1.0, ALTITUDE, 0.0, err_max)) {
                last_request = ros::Time::now();
                should_hover = true;
                hover_start_time = ros::Time::now();
                hover_x = local_pos.pose.pose.position.x;
                hover_y = local_pos.pose.pose.position.y;
                hover_z = local_pos.pose.pose.position.z;
                hover_yaw = yaw;
                image_processed = false;
                image_received = false;
                mission_num = 10;
            }
            break;
        case 10:  // 二维码识别任务
            if (mission_pos_cruise(28.0, 3.0, ALTITUDE, 0.0, err_max)) {
                last_request = ros::Time::now();
                should_hover = true;
                hover_start_time = ros::Time::now();
                hover_x = local_pos.pose.pose.position.x;
                hover_y = local_pos.pose.pose.position.y;
                hover_z = local_pos.pose.pose.position.z;
                hover_yaw = yaw;
                image_processed = false;
                image_received = false;
                mission_num = 11;
            }
            break;
        case 11:  // 数字识别任务（保留原始任务）
            // 悬停15秒进行数字识别
            if (!should_hover)
            {
                should_hover = true;
                hover_start_time = ros::Time::now();
                hover_x = local_pos.pose.pose.position.x;
                hover_y = local_pos.pose.pose.position.y;
                hover_z = local_pos.pose.pose.position.z;
                hover_yaw = yaw;
                image_processed = false;
                image_received = false;
            }
            if ((ros::Time::now() - hover_start_time).toSec() >= hover_duration) {
                mission_num = 12; // 数字识别完成后进入第一个降落地标
                should_hover = false;
                last_request = ros::Time::now();
            }
            break;
        case 12:  // 第一个降落地标 (35, 3)
            if (mission_pos_cruise(35.0, 3.0, ALTITUDE, 0.0, err_max))
            {
                // 到达第一个降落地标，悬停进行颜色识别
                if (!should_hover)
                {
                    should_hover = true;
                    hover_start_time = ros::Time::now();
                    hover_x = local_pos.pose.pose.position.x;
                    hover_y = local_pos.pose.pose.position.y;
                    hover_z = local_pos.pose.pose.position.z;
                    hover_yaw = yaw;
                    image_processed = false;
                    image_received = false;
                    ROS_INFO("到达第一个降落地标，开始悬停进行颜色识别...");
                }

                // 悬停3秒进行颜色识别
                if ((ros::Time::now() - hover_start_time).toSec() >= 3.0)
                {
                    should_hover = false;
                    if (!landing_colors[0].empty())
                    {
                        ROS_INFO("第一个降落地标颜色识别完成: %s", landing_colors[0].c_str());
                    }
                    else
                    {
                        ROS_WARN("第一个降落地标颜色识别失败或超时");
                    }
                    mission_num = 13;
                    last_request = ros::Time::now();
                }
            }
            break;
        case 13:  // 第二个降落地标 (35, 1)
            if (mission_pos_cruise(35.0, 1.0, ALTITUDE, 0.0, err_max))
            {
                // 到达第二个降落地标，悬停进行颜色识别
                if (!should_hover)
                {
                    should_hover = true;
                    hover_start_time = ros::Time::now();
                    hover_x = local_pos.pose.pose.position.x;
                    hover_y = local_pos.pose.pose.position.y;
                    hover_z = local_pos.pose.pose.position.z;
                    hover_yaw = yaw;
                    image_processed = false;
                    image_received = false;
                    ROS_INFO("到达第二个降落地标，开始悬停进行颜色识别...");
                }

                // 悬停3秒进行颜色识别
                if ((ros::Time::now() - hover_start_time).toSec() >= 3.0)
                {
                    should_hover = false;
                    if (!landing_colors[1].empty())
                    {
                        ROS_INFO("第二个降落地标颜色识别完成: %s", landing_colors[1].c_str());
                    }
                    else
                    {
                        ROS_WARN("第二个降落地标颜色识别失败或超时");
                    }
                    mission_num = 14;
                    last_request = ros::Time::now();
                }
            }
            break;
        case 14:  // 第三个降落地标 (35, -1)
            if (mission_pos_cruise(35.0, -1.0, ALTITUDE, 0.0, err_max))
            {
                // 到达第三个降落地标，悬停进行颜色识别
                if (!should_hover)
                {
                    should_hover = true;
                    hover_start_time = ros::Time::now();
                    hover_x = local_pos.pose.pose.position.x;
                    hover_y = local_pos.pose.pose.position.y;
                    hover_z = local_pos.pose.pose.position.z;
                    hover_yaw = yaw;
                    image_processed = false;
                    image_received = false;
                    ROS_INFO("到达第三个降落地标，开始悬停进行颜色识别...");
                }

                // 悬停3秒进行颜色识别
                if ((ros::Time::now() - hover_start_time).toSec() >= 3.0)
                {
                    should_hover = false;
                    if (!landing_colors[2].empty())
                    {
                        ROS_INFO("第三个降落地标颜色识别完成: %s", landing_colors[2].c_str());
                    }
                    else
                    {
                        ROS_WARN("第三个降落地标颜色识别失败或超时");
                    }
                    mission_num = 15;  // 进入颜色比较阶段
                    last_request = ros::Time::now();
                }
            }
            break;
        case 15:  // 颜色比较和决策
            // 比较三个降落地标颜色与起飞地标颜色
            color_compare_result = compareLandmarkColors();

            if (color_compare_result >= 0 && color_compare_result < 3) {
                // 找到匹配的降落地标，飞往该点
                float target_x, target_y;
                if (color_compare_result == 0) {
                    target_x = 35.0; target_y = 3.0;
                }
                else if (color_compare_result == 1) {
                    target_x = 35.0; target_y = 1.0;
                }
                else {
                    target_x = 35.0; target_y = -1.0;
                }

                ROS_INFO("飞往匹配的降落地标: (%.1f, %.1f)", target_x, target_y);

                if (mission_pos_cruise(target_x, target_y, ALTITUDE, 0.0, err_max)) {
                    mission_num = 16;  // 到达匹配点，准备降落
                    last_request = ros::Time::now();
                }
            }
            else {
                // 没有找到匹配，选择最后一个点作为默认降落点
                ROS_WARN("未找到颜色匹配的降落地标，使用默认降落点(35, -1)");
                if (mission_pos_cruise(35.0, -1.0, ALTITUDE, 0.0, err_max)) {
                    mission_num = 16;  // 到达默认点，准备降落
                    last_request = ros::Time::now();
                }
            }
            break;
        case 16:  // 降落
            // 执行原来的降落逻辑
            mavros_msgs::SetMode land_mode;  // 这里添加分号修复编译错误
            land_mode.request.custom_mode = "AUTO.LAND";
            if (set_mode_client.call(land_mode) && land_mode.response.mode_sent) {
                ROS_INFO("切换到自动降落模式");
            }
            if (!current_state.armed) {
                ROS_INFO("无人机已降落并自动解锁");
                mission_num = -1;
            }
            break;
        }  // switch语句结束

        if (should_hover) {
            if ((ros::Time::now() - hover_start_time).toSec() < hover_duration) {
                setpoint_raw.type_mask = 0b111111111000;
                setpoint_raw.coordinate_frame = 1;
                setpoint_raw.position.x = hover_x;
                setpoint_raw.position.y = hover_y;
                setpoint_raw.position.z = hover_z;
                setpoint_raw.yaw = hover_yaw;
            }
            else {
                should_hover = false;
            }
        }

        mavros_setpoint_pos_pub.publish(setpoint_raw);
        ros::spinOnce();
        rate.sleep();

        if (mission_num == -1) {
            exit(0);
        }
    } // 结束 while (ros::ok()) 循环

    return 0;
} // 结束 main 函数