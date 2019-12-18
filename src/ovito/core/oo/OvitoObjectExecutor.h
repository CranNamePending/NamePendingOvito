////////////////////////////////////////////////////////////////////////////////////////
//
//  Copyright 2017 Alexander Stukowski
//
//  This file is part of OVITO (Open Visualization Tool).
//
//  OVITO is free software; you can redistribute it and/or modify it either under the
//  terms of the GNU General Public License version 3 as published by the Free Software
//  Foundation (the "GPL") or, at your option, under the terms of the MIT License.
//  If you do not alter this notice, a recipient may use your version of this
//  file under either the GPL or the MIT License.
//
//  You should have received a copy of the GPL along with this program in a
//  file LICENSE.GPL.txt.  You should have received a copy of the MIT License along
//  with this program in a file LICENSE.MIT.txt
//
//  This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY KIND,
//  either express or implied. See the GPL or the MIT License for the specific language
//  governing rights and limitations.
//
////////////////////////////////////////////////////////////////////////////////////////

#pragma once


#include <ovito/core/Core.h>

namespace Ovito { OVITO_BEGIN_INLINE_NAMESPACE(ObjectSystem)

/**
 * \brief An executor that can be used with Future<>::then() which runs the closure
 *        routine in the context (and in the thread) of this OvitoObject.
 */
class OVITO_CORE_EXPORT OvitoObjectExecutor
{
private:

	/// Helper class that is used by this executor to transmit a callable object
	/// to the UI thread where it is executed in the context on an OvitoObject.
	class OVITO_CORE_EXPORT WorkEventBase : public QEvent
	{
	protected:

		/// Constructor.
		explicit WorkEventBase(const OvitoObject* obj);

		/// Determines whether work can be executed in the context of the OvitoObject or not.
		bool needToCancelWork() const;

		/// Activates the original execution context under which the work was submitted.
		void activateExecutionContext();

		/// Restores the execution context as it was before the work was executed.
		void restoreExecutionContext();

		/// Weak pointer to the OvitoObject which provides the context for the work
		/// to perform.
		QPointer<OvitoObject> _obj;

		/// The execution context (interactive or scripting) under which the work has been submitted.
		int _executionContext;
	};

	/// Helper class that is used by this executor to transmit a callable object
	/// to the UI thread where it is executed in the context on an OvitoObject.
	template<typename F>
	class WorkEvent : public WorkEventBase
	{
	public:

		/// Constructor.
		WorkEvent(const OvitoObject* obj, F&& callable) :
			WorkEventBase(obj), _callable(std::move(callable)) {}

		/// Destructor.
		virtual ~WorkEvent() {
			// Qt events should only be destroyed in the main thread.
			OVITO_ASSERT(QCoreApplication::closingDown() || QThread::currentThread() == QCoreApplication::instance()->thread());
			/// Activate the original execution context under which the work was submitted.
			activateExecutionContext();
			// Execute the work function.
			std::move(_callable)(needToCancelWork());
			/// Restore the execution context as it was before the work was executed.
			restoreExecutionContext();
		}

	private:
		F _callable;
	};

public:

	class OVITO_CORE_EXPORT Work
	{
	public:
		Work(std::unique_ptr<WorkEventBase> event) : _event(std::move(event)) {}
		Work(Work&& other) = default;
		~Work() { OVITO_ASSERT(!_event); }

		// Need to implement copy constructor and copy assignement operator, because std::function requires them.
		// However, a Work object cannot be copied, only moved. We help ourselves by moving the internal event pointer.
		// Note that the copy source becomes invalid as a result.
		Work(const Work& other) noexcept : _event(std::move(const_cast<Work&>(other)._event)) {}

		Work& operator=(const Work& other) noexcept {
			_event = std::move(const_cast<Work&>(other)._event);
			return *this;
		}

		void operator()();
		void post() &&;
	private:
		std::unique_ptr<WorkEventBase> _event;
	};

public:

	/// \brief Constructor.
	OvitoObjectExecutor(const OvitoObject* obj) noexcept : _obj(obj) { OVITO_ASSERT(obj); }

	/// \brief Create some work that can be submitted for execution later.
	template<typename F>
	Work createWork(F&& f) {
		OVITO_ASSERT(_obj != nullptr);
		return Work(std::make_unique<WorkEvent<F>>(_obj, std::move(f)));
	}

	/// Returns the OvitoObject this executor is associated with.
	/// Work submitted to this executor will be executed in the context of the OvitoObject.
	const OvitoObject* object() const { return _obj; }

	/// Returns the unique Qt event type ID used by this class to schedule asynchronous work.
	static QEvent::Type workEventType() {
		static const int _workEventType = QEvent::registerEventType();
		return static_cast<QEvent::Type>(_workEventType);
	}

private:

	const OvitoObject* _obj = nullptr;

	friend class Application;
};

OVITO_END_INLINE_NAMESPACE
}	// End of namespace
