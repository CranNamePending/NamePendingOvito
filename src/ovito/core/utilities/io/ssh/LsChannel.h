///////////////////////////////////////////////////////////////////////////////
//
//  Copyright (2018) Alexander Stukowski
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

#include <ovito/core/Core.h>
#include "ProcessChannel.h"

namespace Ovito { namespace Ssh {

class LsChannel : public ProcessChannel
{
    Q_OBJECT

public:

    /// Constructor.
    explicit LsChannel(SshConnection* connection, const QString& location);

    /// Returns the directory listing received from remote host.
    const QStringList& directoryListing() const { return _directoryListing; }

Q_SIGNALS:

    /// This signal is generated before transmission of a directory listing begins.
    void receivingDirectory();

    /// This signal is generated after a directory listing has been fully transmitted.
    void receivedDirectoryComplete(const QStringList& listing);

private Q_SLOTS:

    /// Is called whenever data arrives from the remote process.
    void processData();

private:

    QStringList _directoryListing;
};


} // End of namespace
} // End of namespace