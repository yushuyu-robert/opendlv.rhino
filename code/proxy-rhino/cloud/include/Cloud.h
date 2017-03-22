/**
 * Copyright (C) 2016 Christian Berger
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
 */

#ifndef PROXY_RHINO_CLOUD_H
#define PROXY_RHINO_CLOUD_H

#include <map>
#include <memory>
#include <string>

#include <opendavinci/odcore/base/module/TimeTriggeredConferenceClientModule.h>
#include <opendavinci/odcore/io/tcp/TCPConnection.h>

namespace opendlv {
namespace proxy {
namespace rhino {

class Cloud : public odcore::base::module::TimeTriggeredConferenceClientModule {
   public:
    Cloud(int32_t const &, char **);
    Cloud(Cloud const &) = delete;
    Cloud &operator=(Cloud const &) = delete;
    virtual ~Cloud();

   private:
    void updateValues(std::string const &);
    void nextContainer(odcore::data::Container &);
    virtual void setUp();
    virtual void tearDown();
    virtual odcore::data::dmcp::ModuleExitCodeMessage::ModuleExitCode body();

    std::map<std::string, std::string> m_outbound;
    std::shared_ptr<odcore::io::tcp::TCPConnection> m_connection;
};

}
}
}

#endif
