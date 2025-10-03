#include "armor.hpp"



cv::Mat process_img(const cv::Mat &img)
{
    std::vector<cv::Mat> channels;
    cv::Mat frame,thresh_img,binary_img;
    cv::split(img, channels); //通道分离BGR
    cv::GaussianBlur(channels[RED_LIGHTBARS], frame, cv::Size(5, 5), 0); //高斯模糊，去除细小噪点
    cv::threshold(frame, thresh_img, 120, 255, cv::THRESH_BINARY);   //二值化
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5)); 
    cv::morphologyEx(thresh_img, binary_img, cv::MORPH_OPEN, kernel); //开运算，去除灯条边缘噪点

    cv::imshow("binary_img",binary_img);
    return binary_img;
   
}
cv::Point2f getLineIntersection(const std::pair<cv::Point2f, cv::Point2f>& line1,
                            const std::pair<cv::Point2f, cv::Point2f>& line2) {
    // 提取直线1的两点坐标
    float x1 = line1.first.x, y1 = line1.first.y;
    float x2 = line1.second.x, y2 = line1.second.y;
    // 提取直线2的两点坐标
    float x3 = line2.first.x, y3 = line2.first.y;
    float x4 = line2.second.x, y4 = line2.second.y;

    // 直线1的一般式参数
    float A1 = y2 - y1;
    float B1 = x1 - x2;
    float C1 = x2 * y1 - x1 * y2;

    // 直线2的一般式参数
    float A2 = y4 - y3;
    float B2 = x3 - x4;
    float C2 = x4 * y3 - x3 * y4;

    // 计算分母 D
    float D = A1 * B2 - A2 * B1;

    // 平行或重合返回(-1, -1)
    if (fabs(D) < 1e-6) {
        return cv::Point2f(-1, -1);  // 用(-1,-1)表示无交点
    }

    // 计算并返回交点
    return cv::Point2f(
        (B1 * C2 - B2 * C1) / D,
        (A2 * C1 - A1 * C2) / D
    );
}


double Distance(cv::Point2f a, cv::Point2f b) {
		return sqrt((a.x - b.x) * (a.x - b.x) +
			(a.y - b.y) * (a.y - b.y));
	}
void detect_lightbar(const cv::Mat& binary_img, const cv::Mat& img)
{
    std::vector<lightbars> lightbars_box;
    std::vector<lightbars> lightbars_res; 
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    // std::vector<cv::Point2f> vertices;
    cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    std::cout << "contours:" << contours.size() << std::endl; 

    for(const auto contour : contours)
    {
        auto area = cv::contourArea(contour);
        if(area < 6)  continue;

        cv::RotatedRect minRect = cv::minAreaRect(contour);
        //minRect.points(vertices); 
		if (minRect.size.width > minRect.size.height) {
			minRect.angle += 90; 
			float t = minRect.size.width;
			minRect.size.width = minRect.size.height;
			minRect.size.height = t;
		}//转换矩形的width和height


        
        
        float length = std::max(minRect.size.width, minRect.size.height); //灯条长度
        float width = std::min(minRect.size.width, minRect.size.height);  //灯条宽度
        std::cout << "height: " << minRect.size.height << ", width: " << minRect.size.width << std::endl;
        float lw_ratio = length / width; //长宽比
        std::cout << "angle: " << minRect.angle << std::endl;

        std::cout << "lw_ratio: " << lw_ratio << ", length: " << length << ", width: " << width << std::endl;

        if (lw_ratio < 3.0 || lw_ratio > 9.0) continue; //长宽比过滤，灯条应该是细长的
        lightbars_box.push_back(lightbars(minRect));
    }

    std::cout << "lightbars_box_size :"  << lightbars_box.size() << std::endl; // 添加调试信息

    
    
    for(int i = 0; i < lightbars_box.size(); i++)
    {
        for(int j = i + 1; j < lightbars_box.size(); j++)
        {
            lightbars &left = lightbars_box[i]; //左灯条
            lightbars &right = lightbars_box[j]; //右灯条
            std::cout << "left_angle: " << left.angle << ", right_angle: " << right.angle << std::endl;
            

            float angle_gap = std::abs(left.angle - right.angle); //角度差
            if(angle_gap > 30.0) 
            {  
                angle_gap = 180 - angle_gap;

            }

            
            
            float length_gap_ratio = std::abs(left.length - right.length) / std::max(left.length, right.length); //长度差比
            
           

            float meanLen = (left.length + right.length) / 2;  //平均长度
            float lengap_ratio = std::abs(left.length - right.length) / meanLen;  //长度差比
            float yGap = abs(left.minRect.center.y - right.minRect.center.y);
            float yGap_ratio = yGap / meanLen;
            float xGap = abs(left.minRect.center.x - right.minRect.center.x);
            float xGap_ratio = xGap / meanLen;
            float dis = Distance(left.minRect.center, right.minRect.center); //两灯条中心点距离
            float ratio = dis / meanLen;  //距离比

            double half_height = (left.minRect.size.height + right.minRect.size.height) / 4;

            
            
            // 输出调试信息
            std::cout << "angle_gap = " << angle_gap << ", lengap_ratio = " << lengap_ratio << ", length_gap_ratio = " << length_gap_ratio
                      << ", yGap_ratio = " << yGap_ratio << ", xGap_ratio = " << xGap_ratio << ", ratio = " << ratio << std::endl;
            
            
            if (angle_gap > 30.0 ||
                length_gap_ratio > 3.0 ||
                lengap_ratio > 0.7 ||
                yGap_ratio > 1.5 ||
                xGap_ratio > 9 || xGap_ratio < 0.8 ||
                ratio > 4.5 || ratio < 0.8) 
                {  
                continue;
                }
            
            std::cout << "successful match" << std::endl;


            
            //绘制所有检测到的灯条
            // for(const auto& lightbar : lightbars_box) {
            //     cv::Point2f vertices[4];
            //     lightbar.minRect.points(vertices);
            //     for (int i = 0; i < 4; i++) {
            //         cv::line(img, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 2);
            //         //cv::circle(img, vertices[2], 3, cv::Scalar(255, 0, 0), -1);
            //         //cv::line
            //     }
            // }
            
                std::vector<cv::Point2f> left_vertices(4);
                std::vector<cv::Point2f> right_vertices(4);


                left.minRect.points(left_vertices.data());
                right.minRect.points(right_vertices.data());
                // cv::Point lp1 = left_vertices[0];
                // cv::Point lp2 = left_vertices[1];
                // cv::Point rp1 = right_vertices[2];
                // cv::Point rp2 = right_vertices[3];

                cv::Point left_top = (left_vertices[1] + left_vertices[2]) / 2;
                cv::Point left_bottom = (left_vertices[0] + left_vertices[3]) / 2;
                cv::Point right_top = (right_vertices[1] + right_vertices[2]) / 2;
                cv::Point right_bottom = (right_vertices[0] + right_vertices[3]) / 2;
                if(right_top.y > right_bottom.y) {
                    std::swap(right_top, right_bottom);
                }
                if(left_top.y > left_bottom.y) {
                    std::swap(left_top, left_bottom);
                }

                cv::circle(img, left_top, 5, cv::Scalar(255, 0, 0), -1);
                cv::circle(img, left_bottom, 5, cv::Scalar(255, 0, 0), -1);
                cv::circle(img, right_top, 5, cv::Scalar(255, 0, 0), -1);
                cv::circle(img, right_bottom, 5, cv::Scalar(255, 0, 0), -1);
                //cv::Point center = cv::Point((left.minRect.center.x + right.minRect.center.x) / 2, (left.minRect.center.y + right.minRect.center.y) / 2);
                std::pair<cv::Point, cv::Point> line(left_top, right_bottom);   
                std::pair<cv::Point, cv::Point> line2(left_bottom, right_top);
                //cv::circle(img, center, 5, cv::Scalar(255, 0, 0), -1);

                cv::line(img, left_top, right_bottom, cv::Scalar(0, 255, 0), 1);
                cv::line(img, left_bottom, right_top, cv::Scalar(0, 255, 0), 1);
                cv::Point2f center = getLineIntersection(line, line2);
                cv::circle(img, center, 5, cv::Scalar(255, 0, 0), -1);



            
            left_vertices.clear();
            right_vertices.clear();
            
        }
    }

    
    
}




int main(int argc, char** argv)
{

    
    cv::VideoCapture video("C:\\Users\\ASUS\\Desktop\\lightbar_derection\\vedio\\3.mp4");
    cv::Mat frame;
    while(true)
    {
        video >> frame;
        if(frame.empty())
        {
            std::cout << "Could not read the frame" << std::endl;
            break;
        }
        auto start_time = std::chrono::steady_clock::now();
        cv::Mat processed_img = process_img(frame);
        detect_lightbar(processed_img, frame);
        auto end_time = std::chrono::steady_clock::now();
        auto fps = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        std::cout << "FPS: " << 1000.0 / fps << std::endl;
        cv::imshow("Frame", frame);
        if(cv::waitKey(100) >= 0) break;
    }
    video.release();
    cv::destroyAllWindows();



    return 0;
}