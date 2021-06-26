#include "yost_fair_mutex.hpp"

using namespace yost;

FairMutex::FairMutex()
{
	next = 0;
	current = 0;
}

void FairMutex::lock()
{
	std::unique_lock<decltype(mtx)> lk(mtx);
	const std::size_t request = next++;
	while (request != current)
	{
		condition.wait(lk);
	}
}

bool FairMutex::tryLock()
{
	std::lock_guard<decltype(mtx)> lk(mtx);
	if (next != current)
	{
		return false;
	}
	next++;
	return true;
}

void FairMutex::unlock()
{
	std::lock_guard<decltype(mtx)> lk(mtx);
	current++;
	condition.notify_all();
}