#ifndef TM_DRIVER_UTILITIES
#define TM_DRIVER_UTILITIES

#include <thread>
#include <queue>
#include <mutex>
#include <future>
#include <functional>

enum class TmCommRC {
	ERR = -1,
	OK = 0,
	TIMEOUT,
	NOTREADY,
	NOTCONNECT,
	NOTSENDALL,
	NOVALIDPACK,
};

class DispatchQueue {
  std::mutex qlock;
  std::queue<std::function<void()>> cmdQueue;
  std::condition_variable empty;
  
public:
  void put(std::function<void()> op);
  std::function<void()> take();
};

class ActiveObject {
 private:
  double val;
  DispatchQueue dispatchQueue;
  std::atomic<bool> done;
  std::thread *runnable;
  
 public:
  ActiveObject();
  ~ActiveObject();

  void run() ;
  void set_function(std::function<void()> func) ;
};

template <typename T> 
class DataSetting{
 private:
  T data;

 public:
  T get_data();
  void set_data(T input);
};

template <typename T> 
T DataSetting<T>::get_data(){
  return data;
}

template <typename T> 
void DataSetting<T>::set_data(T input){
  data = input;
}

template <typename T> 
class MultiThreadCache{
 private:
  DataSetting<T> dataSetting;
  ActiveObject active;
  T *inputData;
  T outputData;
  void set_data_fuction();
  void get_data_fuction();

 public:
  void set_catch_data(T &input);
  T get_catch_data();
};

template <typename T> 
void MultiThreadCache<T>::set_data_fuction(){
  dataSetting.set_data(*inputData);
}

template <typename T> 
void MultiThreadCache<T>::get_data_fuction(){
  outputData = dataSetting.get_data();
}

template <typename T> 
void MultiThreadCache<T>::set_catch_data(T &input){
  this->inputData = &input;
  active.set_function(std::bind(&MultiThreadCache::set_data_fuction, this));
}

template <typename T> 
T MultiThreadCache<T>::get_catch_data(){
  active.set_function(std::bind(&MultiThreadCache::get_data_fuction, this));
  return outputData;
}

#endif
