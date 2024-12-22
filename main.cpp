#include <iostream>
#include <vector>
#include <string>
#include <queue>
#include  "monitor.h"
#include "helper.h"
#include "WriteOutput.h"

using namespace std;

struct Bridge {
    int travel_time;
    int max_wait;
};

struct Ferry {
    int travel_time;
    int max_wait;
    int capacity;
};

struct Crossroad {
    int travel_time;
    int max_wait;
};

struct PathSegment {
    char connector_type;
    int connector_id;
    int from_direction;
    int to_direction;
};

struct Car {
    int id;
    int travel_time;
    int path_length;
    vector<PathSegment> path;
};

struct CarInLane {
    struct timespec car_abs_waiting_time;
    int car_id;
};

class NarrowBridgeMonitor : public Monitor {
    //East = 0 , West = 1
    Condition eastBound, westBound;
    Lock nb_lock;
    std::vector<Condition*> bounds;
    int connector_id;
    std::vector<std::queue<CarInLane> > lanes;
    std::vector<int> onBridgeVector;
    int current_direction = -1;
    Bridge bridgeConfig;

public:
    NarrowBridgeMonitor(const Bridge& config, const int& id)
        : eastBound(this), westBound(this), nb_lock(this), bridgeConfig(config), connector_id(id){
            nb_lock.unlock();
            bounds.push_back(&(this->eastBound));
            bounds.push_back(&(this->westBound));
            lanes.resize(2);
            onBridgeVector.resize(2);
            onBridgeVector[0]=0;
            onBridgeVector[1]=0;
        }

    void pass(int direction, int car_id){
        nb_lock.lock();

        struct timespec waitTime;
        clock_gettime(CLOCK_REALTIME, &waitTime);
        waitTime.tv_sec += bridgeConfig.max_wait / 1000;
        waitTime.tv_nsec += (bridgeConfig.max_wait % 1000) * 1000000;
        waitTime.tv_sec += waitTime.tv_nsec / 1000000000;
        waitTime.tv_nsec %= 1000000000;

        CarInLane car;
        car.car_id = car_id;
        car.car_abs_waiting_time = waitTime;
        lanes[direction].push(car);

        WriteOutput(car_id,'N',connector_id,ARRIVE);

        nb_lock.unlock();
        while(1){
            nb_lock.lock();
            if(direction==current_direction){
                if(onBridgeVector[!direction]>0){
                    bounds[direction]->wait();
                    nb_lock.unlock();
                }
                else if(lanes[direction].front().car_id==car_id){
                    nb_lock.unlock();
                    if(onBridgeVector[direction]>0) sleep_milli(PASS_DELAY);

                    nb_lock.lock();
                    WriteOutput(car_id,'N',connector_id,START_PASSING);
                    lanes[direction].pop();
                    onBridgeVector[direction]++;
                    nb_lock.unlock();

                    bounds[direction]->notifyAll();

                    sleep_milli(bridgeConfig.travel_time);

                    nb_lock.lock();
                    WriteOutput(car_id,'N',connector_id,FINISH_PASSING);
                    onBridgeVector[direction]--;
                    if(onBridgeVector[direction]==0){
                        current_direction=direction;
                        bounds[!direction]->notifyAll();
                    }
                    nb_lock.unlock();
                    break;
                }
                else{
                    bounds[direction]->wait();
                    nb_lock.unlock();
                }
            }
            else if(lanes[!direction].empty() && onBridgeVector[!direction]== 0){
                current_direction=direction;
                bounds[direction]->notifyAll();
                nb_lock.unlock();
            }
            else if(lanes[direction].front().car_id==car_id){
                if(bounds[direction]->timedwait(&lanes[direction].front().car_abs_waiting_time)==ETIMEDOUT){
                    struct timespec waitTime;
                    clock_gettime(CLOCK_REALTIME, &waitTime);
                    waitTime.tv_sec += bridgeConfig.max_wait / 1000;
                    waitTime.tv_nsec += (bridgeConfig.max_wait % 1000) * 1000000;
                    waitTime.tv_sec += waitTime.tv_nsec / 1000000000;
                    waitTime.tv_nsec %= 1000000000;
                    lanes[!direction].front().car_abs_waiting_time = waitTime;
                    current_direction=direction;
                    bounds[direction]->notifyAll();
                    bounds[!direction]->notifyAll();
                }

                nb_lock.unlock();
            }
            else{
                bounds[direction]->wait();
                nb_lock.unlock();
            }
        }
    }
};

class FerryMonitor : public Monitor {
    Condition ferry0, ferry1;
    Condition ferryLane0, ferryLane1;
    Lock f_lock;
    std::vector<Condition*> inFerry;
    std::vector<Condition*> waitFerryLane;
    int connector_id;
    std::vector<std::queue<CarInLane> > ferryQueues;
    Ferry ferryConfig;
    std::vector<bool> capacityFull;

public:
    FerryMonitor(const Ferry& config, const int& id)
        : ferry0(this), ferry1(this), f_lock(this), ferryLane0(this), ferryLane1(this), ferryConfig(config), connector_id(id){
            f_lock.unlock();
            inFerry.push_back(&(this->ferry0));
            inFerry.push_back(&(this->ferry1));
            waitFerryLane.push_back(&(this->ferryLane0));
            waitFerryLane.push_back(&(this->ferryLane1));
            ferryQueues.resize(2);
            capacityFull.assign(2, false);
        }

    void pass(int direction, int car_id){
        f_lock.lock();
        struct timespec waitTime;
        clock_gettime(CLOCK_REALTIME, &waitTime);
        waitTime.tv_sec += ferryConfig.max_wait / 1000;
        waitTime.tv_nsec += (ferryConfig.max_wait % 1000) * 1000000;
        waitTime.tv_sec += waitTime.tv_nsec / 1000000000;
        waitTime.tv_nsec %= 1000000000;

        CarInLane car;
        car.car_id = car_id;
        car.car_abs_waiting_time = waitTime;

        if(capacityFull[direction]){
            waitFerryLane[direction]->wait();
        }

        WriteOutput(car_id,'F',connector_id,ARRIVE);
        ferryQueues[direction].push(car);
        if(ferryQueues[direction].size()==ferryConfig.capacity){
            capacityFull[direction] = true;
            ferryQueues[direction].pop();
            WriteOutput(car_id,'F',connector_id,START_PASSING);
            f_lock.unlock();

            inFerry[direction]->notifyAll();

            f_lock.lock();
            if(ferryQueues[direction].empty()) {
                capacityFull[direction]=false;
                for(int i=0;i<ferryConfig.capacity;i++) waitFerryLane[direction]->notify();
            }
            f_lock.unlock();

            sleep_milli(ferryConfig.travel_time);
            WriteOutput(car_id,'F',connector_id,FINISH_PASSING);
        }
        else if(ferryQueues[direction].front().car_id==car_id){
            inFerry[direction]->timedwait(&ferryQueues[direction].front().car_abs_waiting_time);
            f_lock.unlock();

            f_lock.lock();
            ferryQueues[direction].pop();
            WriteOutput(car_id,'F',connector_id,START_PASSING);
            f_lock.unlock();

            inFerry[direction]->notifyAll();

            f_lock.lock();
            if(ferryQueues[direction].empty()) {
                capacityFull[direction]=false;
                for(int i=0;i<ferryConfig.capacity;i++) waitFerryLane[direction]->notify();
            }
            f_lock.unlock();

            sleep_milli(ferryConfig.travel_time);
            WriteOutput(car_id,'F',connector_id,FINISH_PASSING);
        }
        else{
            inFerry[direction]->wait();
            f_lock.unlock();


            f_lock.lock();
            ferryQueues[direction].pop();
            WriteOutput(car_id,'F',connector_id,START_PASSING);
            f_lock.unlock();

            f_lock.lock();
            if(ferryQueues[direction].empty()) {
                capacityFull[direction]=false;
                for(int i=0;i<ferryConfig.capacity;i++) waitFerryLane[direction]->notify();
            }
            f_lock.unlock();

            sleep_milli(ferryConfig.travel_time);
            WriteOutput(car_id,'F',connector_id,FINISH_PASSING);
        }
    }
};

class CrossroadMonitor : public Monitor {
    Condition bound0, bound1, bound2, bound3;
    Condition timer0, timer1, timer2, timer3;
    Lock cr_lock;
    std::vector<Condition*> bounds;
    std::vector<Condition*> timers;
    std::vector<std::queue<CarInLane> > lanes;
    std::vector<int> onBridgeVector;
    int current_direction = -1;
    int connector_id;
    bool directionChange = false;
    Crossroad crossroadConfig;
    struct timespec waitTime;

public:
    CrossroadMonitor(const Crossroad& config, const int& id)
        : bound0(this), bound1(this), bound2(this), bound3(this), cr_lock(this),
          timer0(this), timer1(this), timer2(this), timer3(this),
          crossroadConfig(config), connector_id(id){
            cr_lock.unlock();
            bounds.push_back(&(this->bound0));
            bounds.push_back(&(this->bound1));
            bounds.push_back(&(this->bound2));
            bounds.push_back(&(this->bound3));

            timers.push_back(&(this->timer0));
            timers.push_back(&(this->timer1));
            timers.push_back(&(this->timer2));
            timers.push_back(&(this->timer3));

            lanes.resize(4);
            onBridgeVector.resize(4);
            onBridgeVector[0]=0;
            onBridgeVector[1]=0;
            onBridgeVector[2]=0;
            onBridgeVector[3]=0;
        }

    void pass(int direction, int car_id){
        cr_lock.lock();

        clock_gettime(CLOCK_REALTIME, &waitTime);
        waitTime.tv_sec += crossroadConfig.max_wait / 1000;
        waitTime.tv_nsec += (crossroadConfig.max_wait % 1000) * 1000000;
        waitTime.tv_sec += waitTime.tv_nsec / 1000000000;
        waitTime.tv_nsec %= 1000000000;

        CarInLane car;
        car.car_id = car_id;
        car.car_abs_waiting_time = waitTime;
        lanes[direction].push(car);

        WriteOutput(car_id,'C',connector_id,ARRIVE);

        if(current_direction==-1){
            current_direction=direction;
        }

        cr_lock.unlock();
        while(1){
            cr_lock.lock();
            if(direction==current_direction){
                if((onBridgeVector[(current_direction+1)%4]>0||onBridgeVector[(current_direction+2)%4]>0||onBridgeVector[(current_direction+3)%4]>0)){
                    bounds[direction]->wait();
                    cr_lock.unlock();
                }
                else if(lanes[direction].front().car_id==car_id){
                    cr_lock.unlock();
                    if(onBridgeVector[direction]>0) sleep_milli(PASS_DELAY);

                    cr_lock.lock();
                    WriteOutput(car_id,'C',connector_id,START_PASSING);
                    lanes[direction].pop();
                    onBridgeVector[direction]++;
                    cr_lock.unlock();

                    bounds[direction]->notifyAll();

                    sleep_milli(crossroadConfig.travel_time);

                    cr_lock.lock();
                    WriteOutput(car_id,'C',connector_id,FINISH_PASSING);
                    onBridgeVector[direction]--;
                    if(onBridgeVector[direction]==0){
                        if(!lanes[(current_direction+1)%4].empty()){
                            current_direction = (current_direction+1)%4;

                            directionChange = true;
                        }
                        else if(!lanes[(current_direction+2)%4].empty()){
                            current_direction = (current_direction+2)%4;

                            directionChange = true;
                        }
                        else if(!lanes[(current_direction+3)%4].empty()){
                            current_direction = (current_direction+3)%4;

                            directionChange = true;
                        }
                        if(directionChange){
                            clock_gettime(CLOCK_REALTIME, &waitTime);
                            waitTime.tv_sec += crossroadConfig.max_wait / 1000;
                            waitTime.tv_nsec += (crossroadConfig.max_wait % 1000) * 1000000;
                            waitTime.tv_sec += waitTime.tv_nsec / 1000000000;
                            waitTime.tv_nsec %= 1000000000;
                            directionChange = false;
                        }
                        bounds[current_direction]->notifyAll();

                    }
                    cr_lock.unlock();
                    break;
                }
                else{
                    bounds[direction]->wait();
                    cr_lock.unlock();
                }
            }
            else if(lanes[current_direction].empty() && onBridgeVector[current_direction]== 0){
                if(!lanes[(current_direction+1)%4].empty()){
                    current_direction = (current_direction+1)%4;
                    directionChange = true;
                }
                else if(!lanes[(current_direction+2)%4].empty()){
                    current_direction = (current_direction+2)%4;
                    directionChange = true;
                }
                else if(!lanes[(current_direction+3)%4].empty()){
                    current_direction = (current_direction+3)%4;
                    directionChange = true;
                }
                if(directionChange){
                    clock_gettime(CLOCK_REALTIME, &waitTime);
                    waitTime.tv_sec += crossroadConfig.max_wait / 1000;
                    waitTime.tv_nsec += (crossroadConfig.max_wait % 1000) * 1000000;
                    waitTime.tv_sec += waitTime.tv_nsec / 1000000000;
                    waitTime.tv_nsec %= 1000000000;
                    directionChange = false;
                }
                bounds[current_direction]->notifyAll();

                cr_lock.unlock();
            }
            else if(lanes[direction].front().car_id==car_id){
                if(bounds[direction]->timedwait(&waitTime)==ETIMEDOUT){
                    clock_gettime(CLOCK_REALTIME, &waitTime);
                    waitTime.tv_sec += crossroadConfig.max_wait / 1000;
                    waitTime.tv_nsec += (crossroadConfig.max_wait % 1000) * 1000000;
                    waitTime.tv_sec += waitTime.tv_nsec / 1000000000;
                    waitTime.tv_nsec %= 1000000000;

                    if(!lanes[(current_direction+1)%4].empty()){
                        current_direction = (current_direction+1)%4;
                    }
                    else if(!lanes[(current_direction+2)%4].empty()){
                        current_direction = (current_direction+2)%4;
                    }
                    else if(!lanes[(current_direction+3)%4].empty()){
                        current_direction = (current_direction+3)%4;
                    }
                    bounds[0]->notifyAll();
                    bounds[1]->notifyAll();
                    bounds[2]->notifyAll();
                    bounds[3]->notifyAll();
                }
                cr_lock.unlock();
            }
            else{
                bounds[direction]->wait();
                cr_lock.unlock();
            }
        }
    }
};

int NN, FN, CN, carNum;
vector<Bridge> bridges;
vector<Ferry> ferries;
vector<Crossroad> crossroads;
vector<Car> cars;

vector<NarrowBridgeMonitor*> nb_monitors;
vector<FerryMonitor*> f_monitors;
vector<CrossroadMonitor*> cr_monitors;


void initializeMonitors(){
    nb_monitors.clear();
    for(unsigned i=0; i < bridges.size();++i){
        Bridge& b=bridges[i];
        NarrowBridgeMonitor* nm = new NarrowBridgeMonitor(bridges[i],i);
        nb_monitors.push_back(nm);
    }
    f_monitors.clear();
    for(unsigned i=0; i < ferries.size();++i){
        FerryMonitor* fm = new FerryMonitor(ferries[i],i);
        f_monitors.push_back(fm);
    }
    cr_monitors.clear();
    for(unsigned i=0; i < crossroads.size();++i){
        CrossroadMonitor* cm = new CrossroadMonitor(crossroads[i],i);
        cr_monitors.push_back(cm);
    }
}

void parse(){
    std::cin >> NN;
    for (int i = 0; i < NN; ++i) {
        Bridge new_bridge;
        std::cin >> new_bridge.travel_time >> new_bridge.max_wait;
        bridges.push_back(new_bridge);
    }

    std::cin >> FN;
    for (int i = 0; i < FN; ++i) {
        Ferry new_ferry;
        std::cin >> new_ferry.travel_time >> new_ferry.max_wait >> new_ferry.capacity;
        ferries.push_back(new_ferry);
    }

    std::cin >> CN;
    for (int i = 0; i < CN; ++i) {
        Crossroad new_crossroad;
        crossroads.push_back(new_crossroad);
        std::cin >> crossroads[i].travel_time >> crossroads[i].max_wait;
    }

    std::cin >> carNum;
    for (int i = 0; i < carNum; ++i) {
        Car car;
        std::cin >> car.travel_time >> car.path_length;
        car.path.resize(car.path_length);

        for (int j = 0; j < car.path_length; ++j) {
            std::string connector;
            int from_direction, to_direction;

            std::cin >> connector >> from_direction >> to_direction;

            char connector_type = connector.substr(0, 1)[0];
            int connector_id = std::stoi(connector.substr(1));

            car.path[j].connector_type = connector_type;
            car.path[j].connector_id = connector_id;
            car.path[j].from_direction = from_direction;
            car.path[j].to_direction = to_direction;
        }
        car.id = i;
        cars.push_back(car);
    }
}

void* car_thread(void* arg){
    Car* car = (Car*)arg;

    for(int i=0;i<car->path_length;++i){
        char current_connector_type = car->path[i].connector_type;
        int current_connector_id = car->path[i].connector_id;

        WriteOutput(car->id,current_connector_type,current_connector_id,TRAVEL);

        sleep_milli(car->travel_time);

        switch(current_connector_type)
        {
        case 'N':
            nb_monitors[current_connector_id]->pass(car->path[i].from_direction, car->id);
            break;

        case 'F':
            f_monitors[current_connector_id]->pass(car->path[i].from_direction, car->id);
            break;

        case 'C':
            cr_monitors[current_connector_id]->pass(car->path[i].from_direction, car->id);
            break;

        default:
            break;
        }
    }
}


int main() {
    parse();

    InitWriteOutput();

    pthread_t threads[carNum];

    initializeMonitors();

    for (int i = 0; i < cars.size(); i++) {
        pthread_create(&threads[i], NULL, car_thread, (void *)&cars[i]);
    }

    for (int i = 0; i < cars.size(); i++) {
        pthread_join(threads[i], NULL);
    }
}
