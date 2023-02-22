#include <iostream>
#include <queue>

template <typename T>
class RingBuffer {
	public :
		RingBuffer(size_t size) : size_(size) {}

		void push(T data) {
			if(full()) buffer_.pop();
			buffer_.push(data);
		}

		T pop() {
			if(empty()) throw std::runtime_error("Buffer is empty");
			T data = buffer_.front();
			buffer_.pop();
			return data;
		}

		T front() {
			return buffer_.front();
		}

		bool empty() const {
			return buffer_.empty();
		}

		bool full() const {
			return buffer_.size() == size_;
		}

		size_t size() const {
			return size_;
		}

	private :
		std::queue<T> buffer_;
		size_t size_;
};
