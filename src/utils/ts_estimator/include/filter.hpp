#include <vector>
#include <algorithm>
#include <deque>
#include <Eigen/Core>

namespace Filter{
    struct bias_data
    {
        Eigen::Vector3d position;
        Eigen::Vector3d euler;
        double time;

        bias_data(){
            position = Eigen::Vector3d::Zero();
            euler = Eigen::Vector3d::Zero();
            time = 0.0;
        }

        bias_data operator +(const bias_data& x) const {
            bias_data b;
            b.position = position + x.position;
            b.euler = euler + x.euler;
            b.time = time + x.time;
            return b;
        }
    };

    class MedianFilter1D {
    public:
        void init(size_t window_size){
            window_size_ = window_size;
            // if (window_size % 2 == 0) {
            //     throw std::invalid_argument("Window size must be odd.");
            // }
        }

        double update(double new_val) {
            if (buffer_.size() == window_size_) {// 移除最早数据
                buffer_.pop_front();
            }
            buffer_.emplace_back(new_val);  // 添加新数据

            // 复制数据并排序
            std::vector<double> sorted_buffer(buffer_.begin(), buffer_.end());
            std::sort(sorted_buffer.begin(), sorted_buffer.end());

            // 返回中值
            return sorted_buffer[sorted_buffer.size() / 2];
        }

    private:
        size_t window_size_;
        std::deque<double> buffer_;
    };

    class MedianFilterBias {
    public:
        void init(size_t window_size){
            filter_p_x_.init(window_size);
            filter_p_y_.init(window_size);
            filter_p_z_.init(window_size);
            filter_r_.init(window_size);
            filter_p_.init(window_size);
            filter_y_.init(window_size);
            filter_t_.init(window_size);
        }
    
        bias_data update(const bias_data& b_cur) {
            b_filt.position(0) = filter_p_x_.update(b_cur.position(0));
            b_filt.position(1) = filter_p_y_.update(b_cur.position(1));
            b_filt.position(2) = filter_p_z_.update(b_cur.position(2));
            b_filt.euler(0) = filter_r_.update(b_cur.euler(0));
            b_filt.euler(1) = filter_p_.update(b_cur.euler(1));
            b_filt.euler(2) = filter_y_.update(b_cur.euler(2));
            b_filt.time = filter_t_.update(b_cur.time);
            return b_filt;
        }
    
    private:
        bias_data b_filt;
        MedianFilter1D filter_p_x_, filter_p_y_, filter_p_z_,
                        filter_r_, filter_p_, filter_y_, filter_t_;
    };
}