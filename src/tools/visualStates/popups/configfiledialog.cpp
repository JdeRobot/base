/*
 *  Copyright (C) 1997-2013 JDERobot Developers Team
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, see <http://www.gnu.org/licenses/>.
 *
 *  Authors : Borja Menéndez Moreno <b.menendez.moreno@gmail.com>
 *            José María Cañas Plaza <jmplaza@gsyc.es>
 *
 */

#include "configfiledialog.h"

/*************************************************************
 * CONSTRUCTOR
 *************************************************************/
ConfigFileDialog::ConfigFileDialog ( std::list<IceInterface>& listInterfaces, std::map<std::string, std::string> mapInterfacesHeader ) {
    this->listInterfaces = listInterfaces;
    this->mapInterfacesHeader = mapInterfacesHeader;
    this->row = 2;
}

/*************************************************************
 * DESTRUCTOR
 *************************************************************/
ConfigFileDialog::~ConfigFileDialog () {}

/*************************************************************
 * POPUP INITIALIZER
 *************************************************************/
void ConfigFileDialog::init () {
    bool fine = true;
	// Load the GtkBuilder file and instantiate its widgets:
    Glib::RefPtr<Gtk::Builder> refBuilder = Gtk::Builder::create();
    const std::string gladepath = resourcelocator::findGladeFile("configfile.glade");
    try{
        refBuilder->add_from_file(gladepath);
    } catch (const Glib::FileError& ex) {
        std::cerr << BEGIN_RED << "FileError: " << ex.what() << END_COLOR << std::endl;
        fine = false;
    } catch(const Glib::MarkupError& ex) {
        std::cerr << BEGIN_RED << "MarkupError: " << ex.what() << END_COLOR << std::endl;
        fine = false;
    } catch(const Gtk::BuilderError& ex) {
        std::cerr << BEGIN_RED << "BuilderError: " << ex.what() << END_COLOR << std::endl;
        fine = false;
    }

    if (fine) {
        refBuilder->get_widget("dialog_configfile", this->dialog);

        refBuilder->get_widget("label_serverType", this->label_serverType);
        refBuilder->get_widget("label_name", this->label_name);
        refBuilder->get_widget("label_proxyName", this->label_proxyName);
        refBuilder->get_widget("label_topic", this->label_topic);
        refBuilder->get_widget("label_ip", this->label_ip);
        refBuilder->get_widget("label_port", this->label_port);
        refBuilder->get_widget("label_interface", this->label_interface);

        refBuilder->get_widget("grid", this->grid);
        refBuilder->get_widget("combobox_serverType", this->combobox_serverType);
        refBuilder->get_widget("entry_name", this->entry_name);
        refBuilder->get_widget("entry_proxyName", this->entry_proxyName);
        refBuilder->get_widget("entry_topic", this->entry_topic);
        refBuilder->get_widget("entry_ip", this->entry_ip);
        refBuilder->get_widget("entry_port", this->entry_port);
        refBuilder->get_widget("combobox_interface", this->combobox_interface);
        refBuilder->get_widget("button_confirm", this->button_confirm);

        refBuilder->get_widget("button_accept", this->button_accept);
        refBuilder->get_widget("button_cancel", this->button_cancel);

        this->refServerTypeTreeModel = Gtk::ListStore::create(this->serverTypeColumns);
        this->combobox_serverType->set_model(this->refServerTypeTreeModel);
        /*
        Gtk::TreeModel::Row row1 = *(this->refServerTypeTreeModel->append());
        row1[serverTypeColumns.colValue] = INACTIVE;
        row1[serverTypeColumns.colName] = "INACTIVE";
         */

        Gtk::TreeModel::Row row1 = *(this->refServerTypeTreeModel->append());
        row1[serverTypeColumns.colValue] = ICE;
        row1[serverTypeColumns.colName] = "ICE";

        row1 = *(this->refServerTypeTreeModel->append());
        row1[serverTypeColumns.colValue] = ROS;
        row1[serverTypeColumns.colName] = "ROS";

        this->combobox_serverType->pack_start(this->serverTypeColumns.colName);
        this->combobox_serverType->set_active(0);

        this->combobox_serverType->signal_changed().connect( sigc::mem_fun(*this, &ConfigFileDialog::onServerTypeChanged) );

        this->refTreeModel = Gtk::ListStore::create(this->m_Columns);
        this->combobox_interface->set_model(this->refTreeModel);

        Gtk::TreeModel::Row row;
        for ( std::map<std::string, std::string>::iterator mapIterator = this->mapInterfacesHeader.begin();
                mapIterator != this->mapInterfacesHeader.end(); mapIterator++ ) {
            row = *(this->refTreeModel->append());
            row[m_Columns.m_col_name] = mapIterator->first;
        }

        this->combobox_interface->pack_start(m_Columns.m_col_name);

        for ( std::list<IceInterface>::iterator listInterfaceIterator = this->listInterfaces.begin();
                listInterfaceIterator != this->listInterfaces.end(); listInterfaceIterator++ ) {
            ServerType serverType = listInterfaceIterator->getServerType();
            Gtk::Label* label_serverType;
            if (serverType == ICE) {
                showIceLabels();
                label_serverType = Gtk::manage(new Gtk::Label("ICE"));
            } else if (serverType == ROS) {
                showRosLabels();
                label_serverType = Gtk::manage(new Gtk::Label("ROS"));
            }
            Gtk::Label* label_name = Gtk::manage(new Gtk::Label(listInterfaceIterator->getName()));
            Gtk::Label* label_proxyName = Gtk::manage(new Gtk::Label(listInterfaceIterator->getProxyName()));
            Gtk::Label* label_topic = Gtk::manage(new Gtk::Label(listInterfaceIterator->getRosTopic()));
            Gtk::Label* label_ip = Gtk::manage(new Gtk::Label(listInterfaceIterator->getIp()));
            Gtk::Label* label_port = Gtk::manage(new Gtk::Label(listInterfaceIterator->getPort()));
            Gtk::Label* label_interface = Gtk::manage(new Gtk::Label(listInterfaceIterator->getInterface()));
            Gtk::Button* button = Gtk::manage(new Gtk::Button("Delete"));

            button->signal_clicked().connect(sigc::bind(
                            sigc::mem_fun(this, &ConfigFileDialog::on_button_delete), this->row));

            this->grid->attach(*label_serverType, 0, this->row, 1,1);
            this->grid->attach(*label_name, 1, this->row, 1, 1);
            this->grid->attach(*label_proxyName, 2, this->row, 1, 1);
            this->grid->attach(*label_topic, 3, this->row, 1, 1);
            this->grid->attach(*label_ip, 4, this->row, 1, 1);
            this->grid->attach(*label_port, 5, this->row, 1, 1);
            this->grid->attach(*label_interface, 6, this->row, 1, 1);
            this->grid->attach(*button, 7, this->row, 1, 1);

            this->row++;
        }

        this->grid->show_all_children();

        this->button_confirm->signal_clicked().connect(sigc::mem_fun(this,
                                        &ConfigFileDialog::on_button_confirm));
        this->button_accept->signal_clicked().connect(sigc::mem_fun(this,
                                        &ConfigFileDialog::on_button_accept));
        this->button_cancel->signal_clicked().connect(sigc::mem_fun(this,
                                        &ConfigFileDialog::on_button_cancel));

        /*
        Gtk::Label* label;
        refBuilder->get_widget("label_topic", label);
        label->set_visible(false);
        refBuilder->get_widget("entry_topic", entry_topic);
        entry_topic->set_visible(false);
         */
        showIceLabels();
        this->dialog->show_now();
    }
}

/*************************************************************
 * PRIVATE METHODS
 *************************************************************/
void ConfigFileDialog::on_button_confirm () {
    Glib::ustring name;
    Gtk::TreeModel::iterator iter = this->combobox_interface->get_active();
    if (iter) {
        Gtk::TreeModel::Row row = *iter;
        name = row[m_Columns.m_col_name];
    } else
        std::cout << "Invalid iter" << std::endl;

    Gtk::TreeModel::iterator serverTypeIter = this->combobox_serverType->get_active();
    int serverType;
    Glib::ustring serverTypeName;
    if (serverTypeIter) {
        Gtk::TreeModel::Row row = *serverTypeIter;
        serverType = row[serverTypeColumns.colValue];
        serverTypeName = row[serverTypeColumns.colName];
    } else
        std::cout << "Invalid server type iter" << std::endl;
    IceInterface iceinterface((ServerType)serverType, std::string(this->entry_name->get_text()),
                                std::string(this->entry_proxyName->get_text()),
                              std::string(this->entry_topic->get_text()),
                                std::string(this->entry_ip->get_text()),
                                std::string(this->entry_port->get_text()),
                                std::string(name));
    std::cout << "servertype:" << iceinterface.getServerType() << std::endl;
    this->listInterfaces.push_back(iceinterface);

    Gtk::Label* label_serverType = Gtk::manage(new Gtk::Label(serverTypeName));
    Gtk::Label* label_name = Gtk::manage(new Gtk::Label(this->entry_name->get_text()));
    Gtk::Label* label_proxyName = Gtk::manage(new Gtk::Label(this->entry_proxyName->get_text()));
    Gtk::Label* label_topic = Gtk::manage(new Gtk::Label(this->entry_topic->get_text()));
    Gtk::Label* label_ip = Gtk::manage(new Gtk::Label(this->entry_ip->get_text()));
    Gtk::Label* label_port = Gtk::manage(new Gtk::Label(this->entry_port->get_text()));
    Gtk::Label* label_interface = Gtk::manage(new Gtk::Label(name));
    Gtk::Button* button = Gtk::manage(new Gtk::Button("Delete"));

    button->signal_clicked().connect(sigc::bind(
                        sigc::mem_fun(this, &ConfigFileDialog::on_button_delete), this->row));

    this->grid->attach(*label_serverType, 0, this->row, 1, 1);
    this->grid->attach(*label_name, 1, this->row, 1, 1);
    this->grid->attach(*label_proxyName, 2, this->row, 1, 1);
    this->grid->attach(*label_topic, 3, this->row, 1,1);
    this->grid->attach(*label_ip, 4, this->row, 1, 1);
    this->grid->attach(*label_port, 5, this->row, 1, 1);
    this->grid->attach(*label_interface, 6, this->row, 1, 1);
    this->grid->attach(*button, 7, this->row, 1, 1);

    this->grid->show_all_children();

    this->entry_name->set_text("");
    this->entry_proxyName->set_text("");
    this->entry_topic->set_text("");
    this->entry_ip->set_text("");
    this->entry_port->set_text("");

    this->row++;
}

void ConfigFileDialog::on_button_delete ( int row ) {
    std::map<int, std::string> mapNames;

    for ( int i = 0; i < 8; i++ ) {
        std::cerr << "i: " << i << std::endl;
        if (i != 7)
            mapNames[i] = std::string(((Gtk::Label*)this->grid->get_child_at(i, row))->get_text());
        this->grid->remove(*this->grid->get_child_at(i, row));
    }

    std::list<IceInterface>::iterator listInterfacesIterator = this->listInterfaces.begin();
    while ( (!listInterfacesIterator->equals(mapNames[0], mapNames[1], mapNames[2], mapNames[3], mapNames[4], mapNames[5], mapNames[6])) &&
            (listInterfacesIterator != this->listInterfaces.end()) )
        listInterfacesIterator++;

    if (listInterfacesIterator != this->listInterfaces.end())
        this->listInterfaces.erase(listInterfacesIterator);
}

void ConfigFileDialog::on_button_accept () {
	delete this->dialog;
    this->m_signal.emit(this->listInterfaces);
}

void ConfigFileDialog::on_button_cancel () {
    delete this->dialog;
}

ConfigFileDialog::type_signal ConfigFileDialog::signal_config () {
    return m_signal;
}

void ConfigFileDialog::showIceLabels() {
    //label_proxyName->set_visible(true);
    //label_ip->set_visible(true);
    //label_port->set_visible(true);

    entry_proxyName->set_sensitive(true);
    entry_ip->set_sensitive(true);
    entry_port->set_sensitive(true);

    //label_topic->set_visible(false);
    entry_topic->set_sensitive(false);
}

void ConfigFileDialog::showRosLabels() {
    //label_proxyName->set_visible(false);
    //label_ip->set_visible(false);
    //label_port->set_visible(false);

    entry_proxyName->set_sensitive(false);
    entry_ip->set_sensitive(false);
    entry_port->set_sensitive(false);

    //label_topic->set_visible(true);
    entry_topic->set_sensitive(true);
}

void ConfigFileDialog::onServerTypeChanged() {
    Gtk::TreeModel::iterator iter = this->combobox_serverType->get_active();
    if (iter) {
        Gtk::TreeModel::Row row = *iter;
        if (row) {
            int id = row[serverTypeColumns.colValue];
            if ((ServerType)id == ICE) {
                showIceLabels();
            } else if ((ServerType)id == ROS) {
                showRosLabels();
            }
        }
    }

}