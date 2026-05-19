#include <opencv2/opencv.hpp>
#include <ZXing/ReadBarcode.hpp>
#include <ZXing/BarcodeFormat.hpp>


struct Qrpose{
    cv::Vec3d rvec;
    cv::Vec3d tvec; 
    std::string data; 
    bool valid = false; 
}; 


class Qr_detection{
    public: 
        Qr_detection(double qr_size){
            qr_size_ = qr_size; 

    }

    public: 

        Qrpose estimate_qr_pose(const cv::Mat& frame, const cv::Mat& K, 
                const cv::Mat& dist){
                    
                Qrpose result; 

                cv::Mat gray;
                cv::cvtColor::(frame, gray, cv::COLOR::BGR2GRAY); 
            
                ZXing::ImageView view(gray.data, gray.cols, gray.rows, ZXing::ImageFormat::Lum);
                ZXing::ReaderOptions opts; 
                opts.setFormats(ZXing::BarcodeFormat::QrCode); 
                opts.setTryHarder(true); 
                opts.setTryRotate(true); 
                
                auto barcodes = ZXing::ReadBarcodes(view, opts); 
                if (barcodes.empty()) return result;
                
                const auto& qr = barcodes[0]; 
                result.data = qr.text();    

                auto pos = qr.position(); 
                std::vector<cv::Point2f> img_pts = {
                    {(float)pos.topLeft().x,   (float)pos.topLeft().y},
                    {(float)pos.topRight().x,   (float)pos.topRight().y}, 
                    {(float)pos.bottomRight().x,   (float)pos.bottomRight().y}, 
                    {(float)pos.bottomLeft().x,   (float)pos.bottomLeft().y}
                };  

                cv::cornerSubPix(gray, img_pts, {5, 5}, {-1, -1}, 
                                {cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 0.01}); 
                    

                float h = qr_size / 2.0f; 
                std::vector<cv::Point3f> obj_pts = {
                    {-h, h, 0}, 
                    {h, h, 0},
                    {h, -h, 0},
                    {-h, -h, 0},            
                }; 

                bool ok = cv::solvePnP(
                    obj_pts, img_pts, K, dist, 
                    result.rvec, result.tvec, 
                    false, cv::SOLVEPNP_IPPE_SQUARE
                ); 
                

                result.valid = ok; 
                return result; 
            }

        float qr_size_;  

        


}; 
    