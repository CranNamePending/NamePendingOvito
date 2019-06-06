///////////////////////////////////////////////////////////////////////////////
//
//  Copyright (2017) Alexander Stukowski
//
//  This file is part of OVITO (Open Visualization Tool).
//
//  OVITO is free software; you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation; either version 2 of the License, or
//  (at your option) any later version.
//
//  OVITO is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
///////////////////////////////////////////////////////////////////////////////

#pragma once


#include <core/Core.h>
#include "Task.h"
#include "FutureDetail.h"

namespace Ovito { OVITO_BEGIN_INLINE_NAMESPACE(Util) OVITO_BEGIN_INLINE_NAMESPACE(Concurrency)

/**
 * \brief This type of Task is returned by the Future::then() method and is needed to implement continuation functions.
 */
class OVITO_CORE_EXPORT TrackingTask : public Task
{
public:

	/// Constructor.
	TrackingTask(TaskDependency creatorState) : _creatorState(std::move(creatorState)) {}

#ifdef OVITO_DEBUG
	/// Destructor.
	virtual ~TrackingTask();
#endif

	/// Returns the maximum value for progress reporting.
    virtual qlonglong progressMaximum() const override { return trackedState() ? trackedState()->progressMaximum() : 0; }

	/// Returns the current progress value (in the range 0 to progressMaximum()).
	virtual qlonglong progressValue() const override { return trackedState() ? trackedState()->progressValue() : 0; }

	/// Return the current status text set for this promise.
    virtual QString progressText() const override { return trackedState() ? trackedState()->progressText() : QString(); }

	/// Cancels this promise.
	virtual void cancel() noexcept override;

	/// Marks this promise as fulfilled.
	virtual void setFinished() override;

	/// Returns the promise that created this promise as a continuation.
	const TaskPtr& creatorState() const { return _creatorState.get(); }

	template<typename FC, typename FunctionParams>
	void fulfillWith(FC&& closure, FunctionParams&& params) noexcept
	{
		try {
			// Call the continuation function with the results of the fulfilled promise.
			// Assign the returned future to this tracking promise state.
			auto future = detail::apply(std::forward<FC>(closure), std::forward<FunctionParams>(params));
			setTrackedState(std::move(future._task));
		}
		catch(...) {
			setStarted();
			captureException();
			setFinished();
		}
	}

protected:

	/// Makes this promise state track the given other state.
	void setTrackedState(TaskDependency&& state);

	/// Returns the inner promise state.
	const TaskPtr& trackedState() const {
		return _trackedState.get();
	}

	/// The promise being tracked by this promise state.
	TaskDependency _trackedState;

	/// The promise that created this promise as a continuation.
	TaskDependency _creatorState;

	/// Linked list pointer.
	std::shared_ptr<TrackingTask> _nextInList;

	template<typename... R2> friend class Future;
	friend class Task;
	friend class ProgressiveTask;
};

OVITO_END_INLINE_NAMESPACE
OVITO_END_INLINE_NAMESPACE
}	// End of namespace