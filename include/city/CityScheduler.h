/*
 * Note: This license has also been called the "New BSD License" 
 * or "Modified BSD License".
 * 
 * Copyright (c) 2021 Electronics and Telecommunications Research 
 * Institute All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors 
 *    may be used to endorse or promote products derived from this software 
 *    without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
 * POSSIBILITY OF SUCH DAMAGE.
*/

/**
 * CityScheduler.h
 *
 * $Revision: 765 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#ifndef CITY_SCHEDULER_H
#define CITY_SCHEDULER_H

#include <vector>
#include <queue>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <boost/mpi/communicator.hpp>
#include <boost/shared_ptr.hpp>

#include "CityException.h"

namespace dtsim {

class Functor {
public:
	virtual ~Functor() {}
	virtual void operator()() = 0;
};

template<typename T>
class TemplateMethodFunctor: public Functor {
private:
	void (T::*fptr)();
	T* obj;

public:
	TemplateMethodFunctor(T* _obj, void(T::*_fptr)()) : fptr(_fptr), obj(_obj) {}
	~TemplateMethodFunctor() {}

	void operator()() {
		(obj->*fptr)();
	}
};

class ClassMethodFunctor: public Functor {
private:
	void (*fptr)();

public:
	ClassMethodFunctor(void(*_fptr)()) : fptr(_fptr) {}
	~ClassMethodFunctor() {}

	void operator()() {
		(*fptr)();
	}
};

class Event {
public:
	double tick;
	boost::shared_ptr<Functor> func_ptr;

	virtual ~Event() {}
};

class EventCompare;

class ScheduleEvent {
private:
	Event* event;
	double start;
	double interval;

public:
	friend class EventCompare;

	ScheduleEvent(double start_, double interval_, Event*);
	virtual ~ScheduleEvent();

	bool reschedule(std::priority_queue<ScheduleEvent*, std::vector<ScheduleEvent*>, EventCompare>&);

	double getInterval() {
		return interval;
	}

	Event* getEvent() {
		return event;
	}
};

class EventCompare {
public:
	int operator()(const ScheduleEvent* one, const ScheduleEvent* two) {
		double tick1 = one->event->tick;
		double tick2 = two->event->tick;
		return tick1 > tick2 ? 1 : 0;
	}
};

class CityScheduler;

class EventWorker {
private:
	CityScheduler* scheduler;
	std::mutex mutex_;
	std::condition_variable cv_;
	std::thread* worker;
	ScheduleEvent* event;
	bool ready;
	bool exit;

public:
	std::thread::id tid;
	int id;

public:
	EventWorker(CityScheduler* scheduler_, int id_);
	virtual ~EventWorker();

	void wait_event();
	void wakeup_event(ScheduleEvent* event_);
	void wakeup_exit();

	void operator()();
};

/**
 * \class CityScheduler
 *
 * \brief
 * Agent event scheduler
 */
class CityScheduler {
private:
	std::priority_queue<ScheduleEvent*, std::vector<ScheduleEvent*>, EventCompare> queue;

	boost::mpi::communicator* comm;
	bool go;
	double globalNextTick;
	double localNextTick;
	double currentTick;
	void nextTick();
	std::vector<boost::shared_ptr<Functor>> endEvents;

	int numWorkers;
	std::vector<EventWorker*> workers;

	std::mutex mutex_;
	std::condition_variable cv_;
	int working;
	int workingDone;

	void runSingle();
	void runMulti();

public:
	CityScheduler(boost::mpi::communicator* communicator, int n);
	virtual ~CityScheduler();

	template<typename T>
	void addEvent(double at, T* obj, void(T::*func)());

	template<typename T>
	void addEvent(double start, double interval, T* obj, void(T::*func)());

	template<typename T>
	void addEvent(double start, double interval, int nrepeat, T* obj, void(T::*func)());

	template<typename T>
	void addEndEvent(T* obj, void(T::*func)());

	/// Add event function to execute once at the specified tick
	void addEvent(double at, void(*func)());

	/// Add event function to execute at the specified tick and every interval thereafter
	void addEvent(double start, double interval, void(*func)());

	/// Add event function to execute at the specified tick and every interval thereafter
	void addEvent(double start, double interval, int nrepeat, void(*func)());

	/// Add event function to execute when the simulation ends
	void addEndEvent(void(*func)());

	/**
	 * Schedules the simulation to stop at the specified tick
	 *
	 * @param at the tick at which the simulation should stop
	 */
	void addStopEvent(double at);

	/// Non User API
	void wait_done();

	/// Non User API
	void wakeup_done();

	/// Starts and runs simulation schedule
	void run();

	/// Stops the simulation
	void stop();

	/// Restarts the simulation
	void restart(double stopAt);

	/// Gets the current tick
	double getCurrentTick() const {
		return currentTick;
	}

	/// Gets the number of workers
	int getNumWorkers() const {
		return numWorkers;
	}

	/// Get worker id 
	int getWorkerId() {
		for (auto W : workers) {
			if (W->tid == std::this_thread::get_id()) {
				return W->id;
			}
		}
		return 0;
	}
};

/**
 * Add object's event function to execute once at the specified tick
 *
 * @tparam T the type of object to schedule event
 * @param at
 * @param obj
 * @param func
 */
template<typename T>
void CityScheduler::addEvent(double at, T* obj, void(T::*func)())
{
	if (func == nullptr)
		throw CityException(__func__, "event function null");

	Event* event = new Event();
	event->func_ptr = boost::shared_ptr<Functor>(new TemplateMethodFunctor<T>(obj, func));
	event->tick = at;

	ScheduleEvent* scheduleEvent = new ScheduleEvent(at, 0, event);
	queue.push(scheduleEvent);

	nextTick();
}

/**
 * Add object's event function to execute at the specified tick and every interval thereafter
 *
 * @tparam T the type of object to schedule event
 * @param start
 * @param interval
 * @param obj
 * @param func
 */
template<typename T>
void CityScheduler::addEvent(double start, double interval, T* obj, void(T::*func)())
{
	if (func == nullptr)
		throw CityException(__func__, "event function null");

	Event* event = new Event();
	event->func_ptr = boost::shared_ptr<Functor>(new TemplateMethodFunctor<T>(obj, func));
	event->tick = start;

	ScheduleEvent* scheduleEvent = new ScheduleEvent(start, interval, event);
	queue.push(scheduleEvent);

	nextTick();
}

/**
 * Add object's event function to execute at the specified tick and every interval thereafter
 *
 * @tparam T the type of object to schedule event
 * @param start
 * @param interval
 * @param nrepeat
 * @param obj
 * @param func
 */
template<typename T>
void CityScheduler::addEvent(double start, double interval, int nrepeat,  T* obj, void(T::*func)())
{
	if (func == nullptr)
		throw CityException(__func__, "event function null");

	for (int i = 0; i < nrepeat; i++) {
		Event* event = new Event();
		event->func_ptr = boost::shared_ptr<Functor>(new TemplateMethodFunctor<T>(obj, func));
		event->tick = start;

		ScheduleEvent* scheduleEvent = new ScheduleEvent(start, interval, event);
		queue.push(scheduleEvent);

		nextTick();
	}
}

/**
 * Add object's event function to execute when the simulation ends
 *
 * @tparam T the type of object to schedule event
 * @param obj
 * @param func
 */
template<typename T>
void CityScheduler::addEndEvent(T* obj, void(T::*func)())
{
	if (func == nullptr)
		throw CityException(__func__, "event function null");

	endEvents.push_back(boost::shared_ptr<Functor>(new TemplateMethodFunctor<T>(obj, func)));
}

} /* namespace dtsim */

#endif /* CITY_SCHEDULER_H */
