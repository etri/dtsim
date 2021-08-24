/*                                                                                                                  
 *                                                                                                                   * Note: This license has also been called the "New BSD License" 
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
 * CityScheduler.cpp
 *
 * $Revision: 655 $
 * $LastChangedDate: 2019-07-19 15:18:25 +0900 (Fri, 19 Jul 2019) $
 */

#include "City.h"
#include "CityScheduler.h"

namespace dtsim {

ScheduleEvent::ScheduleEvent(double start_, double interval_, Event* e)
: event(e), start(start_), interval(interval_)
{}

ScheduleEvent::~ScheduleEvent()
{
	delete event;
}

bool ScheduleEvent::reschedule(std::priority_queue<ScheduleEvent*, std::vector<ScheduleEvent*>, EventCompare>& queue)
{
	if (interval > 0) {
		event->tick += interval;
		queue.push(this);
		return true;
	} else {
		return false;
	}
}

EventWorker::EventWorker(CityScheduler* scheduler_, int id_)
: scheduler(scheduler_), ready(false), exit(false)
{
	worker = new std::thread(std::ref(*this));

	tid = worker->get_id();
	id = id_;
}

EventWorker::~EventWorker()
{
	worker->join();
	delete worker;
}

void EventWorker::wait_event()
{
	std::unique_lock<std::mutex> lock(mutex_);
	while (!ready && !exit)
		cv_.wait(lock);
}

void EventWorker::wakeup_event(ScheduleEvent* event_)
{
	{
		std::lock_guard<std::mutex> lock(mutex_);
		event = event_;
		ready = true;
	}
	cv_.notify_one();
}

void EventWorker::wakeup_exit()
{
	{
		std::lock_guard<std::mutex> lock(mutex_);
		exit = true;
	}
	cv_.notify_one();
}

void EventWorker::operator()()
{
	while (!exit) {
		wait_event();

		if (exit)
			break;

		Functor* func = event->getEvent()->func_ptr.get();
		(*func)();

		if (event->getInterval() == 0)
			delete event;

		ready = false;

		scheduler->wakeup_done();
	}

	scheduler->wakeup_done();
}

CityScheduler::CityScheduler(boost::mpi::communicator* communicator, int n)
: queue(), currentTick(0), go(true), comm(communicator), numWorkers(n)
{
	if (numWorkers > 1) {
		for (int i = 0; i < numWorkers; i++) {
			EventWorker* worker = new EventWorker(this, i);
			workers.push_back(worker);
		}
	}
}

CityScheduler::~CityScheduler()
{
	while (!queue.empty()) {
		ScheduleEvent* event = queue.top();
		queue.pop();
		delete event;
	}

	if (numWorkers > 1) {
		std::vector<EventWorker*>::iterator iter = workers.begin();
		while (iter != workers.end()) {
			EventWorker* worker = *iter;
			delete worker;
			iter++;
		}
	}
}

void CityScheduler::nextTick()
{
	if (queue.empty())
		localNextTick = -1;
	else
		localNextTick = queue.top()->getEvent()->tick;
}

void CityScheduler::wait_done()
{
	{
		std::unique_lock<std::mutex> lock(mutex_);
		while (workingDone != working)
			cv_.wait(lock);
	}
	working = 0;
	workingDone = 0;
}

void CityScheduler::wakeup_done()
{
	{
		std::lock_guard<std::mutex> lock(mutex_);
		workingDone++;
	}
	cv_.notify_one();
}

void CityScheduler::runMulti()
{
	while (go) {
		all_reduce(*comm, localNextTick, globalNextTick, boost::mpi::minimum<double>());

		if (localNextTick == globalNextTick) {
			if (!queue.empty()) {
				ScheduleEvent* event = queue.top();
				double next = event->getEvent()->tick;
				currentTick = next;

				std::vector<EventWorker*>::iterator iter = workers.begin();
				std::vector<EventWorker*>::iterator iterEnd = workers.end();
				working = 0;
				workingDone = 0;

				do {
					queue.pop();

					event->reschedule(queue);

					(*iter)->wakeup_event(event);

					working++;

					event = queue.top();
					if (queue.empty() || (event->getEvent()->tick != next) || (++iter == iterEnd))
						wait_done();

					if (queue.empty())
						break;

					if (event->getEvent()->tick != next)
						break;

					if (iter == iterEnd)
						iter = workers.begin();
				} while (event->getEvent()->tick == next);
			}
		}

		nextTick();
	}

	working = numWorkers;
	workingDone = 0;
	std::vector<EventWorker*>::iterator iter = workers.begin();
	std::vector<EventWorker*>::iterator iterEnd = workers.end();
	while (iter != iterEnd) {
		(*iter)->wakeup_exit();
		iter++;
	}

	wait_done();

	for (size_t i = 0; i < endEvents.size(); i++)
		(*endEvents[i])();
}

void CityScheduler::runSingle()
{
	while (go) {
		all_reduce(*comm, localNextTick, globalNextTick, boost::mpi::minimum<double>());

		if (localNextTick == globalNextTick) {
			if (!queue.empty()) {
				ScheduleEvent* event = queue.top();
				double next = event->getEvent()->tick;
				currentTick = next;

				do {
					queue.pop();

					Functor* func = event->getEvent()->func_ptr.get();
					(*func)();

					if (event->reschedule(queue) == false)
						delete event;

					if (queue.empty())
						break;

					event = queue.top();
				} while (event->getEvent()->tick == next);
			}
		}

		nextTick();
	}

	for (size_t i = 0; i < endEvents.size(); i++)
		(*endEvents[i])();
}

void CityScheduler::run()
{
	if (numWorkers <= 1)
		runSingle();
	else
		runMulti();
}

void CityScheduler::addEvent(double at, void(*func)())
{
	if (func == nullptr)
		throw CityException(__func__, "event function null");

	Event* event = new Event();
	event->func_ptr = boost::shared_ptr<Functor>(new ClassMethodFunctor(func));
	event->tick = at;

	ScheduleEvent* scheduleEvent = new ScheduleEvent(at, 0, event);
	queue.push(scheduleEvent);

	nextTick();
}

void CityScheduler::addEvent(double start, double interval, void(*func)())
{
	if (func == nullptr)
		throw CityException(__func__, "event function null");

	Event* event = new Event();
	event->func_ptr = boost::shared_ptr<Functor>(new ClassMethodFunctor(func));
	event->tick = start;

	ScheduleEvent* scheduleEvent = new ScheduleEvent(start, interval, event);
	queue.push(scheduleEvent);

	nextTick();
}

void CityScheduler::addEvent(double start, double interval, int nrepeat, void(*func)())
{
	if (func == nullptr)
		throw CityException(__func__, "event function null");

	for (int i = 0; i < nrepeat; i++) {
		Event* event = new Event();
		event->func_ptr = boost::shared_ptr<Functor>(new ClassMethodFunctor(func));
		event->tick = start;

		ScheduleEvent* scheduleEvent = new ScheduleEvent(start, interval, event);
		queue.push(scheduleEvent);

		nextTick();
	}
}

void CityScheduler::addEndEvent(void(*func)())
{
	if (func == nullptr)
		throw CityException(__func__, "event function null");

	endEvents.push_back(boost::shared_ptr<Functor>(new ClassMethodFunctor(func)));
}

void CityScheduler::addStopEvent(double at)
{
	Event* event = new Event();
	event->func_ptr = boost::shared_ptr<Functor>(new TemplateMethodFunctor<CityScheduler>(this, &CityScheduler::stop));
	event->tick = at;

	ScheduleEvent* scheduleEvent = new ScheduleEvent(at, 0, event);
	queue.push(scheduleEvent);

	nextTick();
}

void CityScheduler::stop()
{
	go = false;

	if (endEvents.empty() == true) {
		std::vector<EventWorker*>::iterator iter = workers.begin();
		std::vector<EventWorker*>::iterator iterEnd = workers.end();
		while (iter != iterEnd) {
			(*iter)->wakeup_exit();
			iter++;
		}
	}
}

void CityScheduler::restart(double stopAt)
{
	go = true;
	currentTick = 0;

	endEvents.clear();

	while (!queue.empty()) {
		ScheduleEvent* event = queue.top();
		queue.pop();
		delete event;
	}

	if (numWorkers > 1) {
		std::vector<EventWorker*>::iterator iter = workers.begin();
		while (iter != workers.end()) {
			EventWorker* worker = *iter;
			delete worker;
			iter++;
		}
		workers.clear();

		for (int i = 0; i < numWorkers; i++) {
			EventWorker* worker = new EventWorker(this, i);
			workers.push_back(worker);
		}
	}

	addStopEvent(stopAt);
}

} /* namespace dtsim */
