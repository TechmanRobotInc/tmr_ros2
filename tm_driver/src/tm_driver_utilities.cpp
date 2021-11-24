#include"../include/tm_driver/tm_driver_utilities.h"

void DispatchQueue::put(std::function<void()> op) {
  std::lock_guard<std::mutex> guard(qlock);
  cmdQueue.push(op);
  empty.notify_one();
}

std::function<void()> DispatchQueue::take() {
  std::unique_lock<std::mutex> lock(qlock);
  empty.wait(lock, [&]{ return !cmdQueue.empty(); });

  std::function<void()> op = cmdQueue.front();
  cmdQueue.pop();
  return op;
}

ActiveObject::ActiveObject() : val(0), done(false) { 
  runnable = new std::thread(&ActiveObject::run, this); 
}
ActiveObject::~ActiveObject() { 
  runnable->join(); 
}

void ActiveObject::run() {
  while (!done) {
   dispatchQueue.take()();
  }
}
void ActiveObject::set_function(std::function<void()> func) {
  dispatchQueue.put(func);
}  
