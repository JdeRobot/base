#include "gui.h"

namespace navigatorCamera {

	Gui::Gui(Sharer *sharer,const std::string& gladeFile) : gtkmain(0, 0)
	{

		this->sharer = sharer;

		std::cout << "Loading glade." << std::endl;

		const std::string gladepath = std::string(GLADE_DIR) + gladeFile;
		refXml = Gnome::Glade::Xml::create(gladepath);

		refXml->get_widget("showWindow", showWindow);
		refXml->get_widget("RGB", RGB);

		refXml->get_widget("txtInfoX", txtInfoX);
		refXml->get_widget("txtInfoY", txtInfoY);
		refXml->get_widget("txtInfoZ", txtInfoZ);
		refXml->get_widget("txtInfoH", txtInfoH);
		refXml->get_widget("txtInfoQ0", txtInfoQ0);
		refXml->get_widget("txtInfoQ1", txtInfoQ1);
		refXml->get_widget("txtInfoQ2", txtInfoQ2);
		refXml->get_widget("txtInfoQ3", txtInfoQ3);

		refXml->get_widget("controlWindow", controlWindow);

		refXml->get_widget("bttnTrlnRight", bttnTrlnRight);
		refXml->get_widget("bttnTrlnLeft", bttnTrlnLeft);
		refXml->get_widget("bttnTrlnFront", bttnTrlnFront);
		refXml->get_widget("bttnTrlnBack", bttnTrlnBack);
		refXml->get_widget("bttnTrlnUp", bttnTrlnUp);
		refXml->get_widget("bttnTrlnDown", bttnTrlnDown);

		refXml->get_widget("bttnRttnRight", bttnRttnRight);
		refXml->get_widget("bttnRttnLeft", bttnRttnLeft);
		refXml->get_widget("bttnRttnUp", bttnRttnUp);
		refXml->get_widget("bttnRttnDown", bttnRttnDown);
		refXml->get_widget("bttnRttnCW", bttnRttnCW);
		refXml->get_widget("bttnRttnACW", bttnRttnACW);

		refXml->get_widget("spnbTrlnStp", spnbTrlnStp);
		refXml->get_widget("spnbRttnStp", spnbRttnStp);

		refXml->get_widget("bttnRstr", bttnRstr);

		bttnTrlnRight->signal_clicked().connect(sigc::mem_fun(this,&Gui::gtk_bttnTrlnRight_onButtonClick));
		bttnTrlnLeft->signal_clicked().connect(sigc::mem_fun(this,&Gui::gtk_bttnTrlnLeft_onButtonClick));
		bttnTrlnFront->signal_clicked().connect(sigc::mem_fun(this,&Gui::gtk_bttnTrlnFront_onButtonClick));
		bttnTrlnBack->signal_clicked().connect(sigc::mem_fun(this,&Gui::gtk_bttnTrlnBack_onButtonClick));
		bttnTrlnUp->signal_clicked().connect(sigc::mem_fun(this,&Gui::gtk_bttnTrlnUp_onButtonClick));
		bttnTrlnDown->signal_clicked().connect(sigc::mem_fun(this,&Gui::gtk_bttnTrlnDown_onButtonClick));

		bttnRttnRight->signal_clicked().connect(sigc::mem_fun(this,&Gui::gtk_bttnRttnRight_onButtonClick));
		bttnRttnLeft->signal_clicked().connect(sigc::mem_fun(this,&Gui::gtk_bttnRttnLeft_onButtonClick));
		bttnRttnUp->signal_clicked().connect(sigc::mem_fun(this,&Gui::gtk_bttnRttnUp_onButtonClick));
		bttnRttnDown->signal_clicked().connect(sigc::mem_fun(this,&Gui::gtk_bttnRttnDown_onButtonClick));
		bttnRttnCW->signal_clicked().connect(sigc::mem_fun(this,&Gui::gtk_bttnRttnCW_onButtonClick));
		bttnRttnACW->signal_clicked().connect(sigc::mem_fun(this,&Gui::gtk_bttnRttnACW_onButtonClick));

		spnbTrlnStp->signal_value_changed().connect(sigc::mem_fun(this,&Gui::gtk_spnbTrlnStp_valueChanged));
		spnbRttnStp->signal_value_changed().connect(sigc::mem_fun(this,&Gui::gtk_spnbRttnStp_valueChanged));

		bttnRstr->signal_clicked().connect(sigc::mem_fun(this,&Gui::gtk_restart_onButtonClick));

		std::cout << "Glade done." << std::endl;

		spnbTrlnStp->set_value(sharer->getTranslationStep());
		spnbRttnStp->set_value(sharer->getRotationStep());

		controlWindow->add_events(Gdk::KEY_PRESS_MASK);
		controlWindow->signal_key_press_event().connect(sigc::mem_fun(this,&Gui::on_key_press_event));

		/*Show window. Note: Set window visibility to false in Glade, otherwise opengl won't work*/
		showWindow->show();
		controlWindow->show();
	}

	Gui::~Gui()
	{
		delete this->showWindow;
		delete this->controlWindow;
	}

	bool Gui::isVisible()
	{
		return showWindow->get_visible() && controlWindow->get_visible();
	}

	void Gui::display()
	{
		cv::Mat image = sharer->getImage();
		Glib::RefPtr<Gdk::Pixbuf> imgBuff =  Gdk::Pixbuf::create_from_data((const guint8*) image.data, Gdk::COLORSPACE_RGB, false, 8, image.cols, image.rows, image.step);
		RGB->clear();
		RGB->set(imgBuff);

		this->showPose3d();

		while ( gtkmain.events_pending() )
			gtkmain.iteration();

		this->showWindow->queue_draw();
	}

	void Gui::showPose3d()
	{
		std::string output;			// String for text information widgets
		std::stringstream sstream;	// Stream for conversion to text

		jderobot::Pose3DDataPtr p3d = this->sharer->getPose3D();

		sstream << p3d->x;
		sstream >> output;	sstream.flush();
		txtInfoX->set_text(Glib::ustring(output));
		output.clear();
		sstream.clear();

		sstream << p3d->y;
		sstream >> output;	sstream.flush();
		txtInfoY->set_text(Glib::ustring(output));
		output.clear();
		sstream.clear();

		sstream << p3d->z;
		sstream >> output;	sstream.flush();
		txtInfoZ->set_text(Glib::ustring(output));
		output.clear();
		sstream.clear();

		sstream << p3d->h;
		sstream >> output;	sstream.flush();
		txtInfoH->set_text(Glib::ustring(output));
		output.clear();
		sstream.clear();

		sstream << p3d->q0;
		sstream >> output;	sstream.flush();
		txtInfoQ0->set_text(Glib::ustring(output));
		output.clear();
		sstream.clear();

		sstream << p3d->q1;
		sstream >> output;	sstream.flush();
		txtInfoQ1->set_text(Glib::ustring(output));
		output.clear();
		sstream.clear();

		sstream << p3d->q2;
		sstream >> output;	sstream.flush();
		txtInfoQ2->set_text(Glib::ustring(output));
		output.clear();
		sstream.clear();

		sstream << p3d->q3;
		sstream >> output;	sstream.flush();
		txtInfoQ3->set_text(Glib::ustring(output));
		output.clear();
		sstream.clear();
	}

	void Gui::gtk_bttnTrlnRight_onButtonClick()
	{
		sharer->changePose3dTranslation(0., -1., 0.);
	}

	void Gui::gtk_bttnTrlnLeft_onButtonClick()
	{
		sharer->changePose3dTranslation(0., 1., 0.);
	}

	void Gui::gtk_bttnTrlnFront_onButtonClick()
	{
		sharer->changePose3dTranslation(1., 0., 0.);
	}

	void Gui::gtk_bttnTrlnBack_onButtonClick()
	{
		sharer->changePose3dTranslation(-1., 0., 0.);
	}

	void Gui::gtk_bttnTrlnUp_onButtonClick()
	{
		sharer->changePose3dTranslation(0., 0., 1.);
	}

	void Gui::gtk_bttnTrlnDown_onButtonClick()
	{
		sharer->changePose3dTranslation(0., 0., -1.);
	}

	void Gui::gtk_bttnRttnRight_onButtonClick()
	{
		sharer->changePose3dRotation(-1., 0., 0.);
	}

	void Gui::gtk_bttnRttnLeft_onButtonClick()
	{
		sharer->changePose3dRotation(1., 0., 0.);
	}

	void Gui::gtk_bttnRttnUp_onButtonClick()
	{
		sharer->changePose3dRotation(0., -1., 0.);
	}

	void Gui::gtk_bttnRttnDown_onButtonClick()
	{
		sharer->changePose3dRotation(0., 1., 0.);
	}

	void Gui::gtk_bttnRttnCW_onButtonClick()
	{
		sharer->changePose3dRotation(0., 0., 1.);
	}

	void Gui::gtk_bttnRttnACW_onButtonClick()
	{
		sharer->changePose3dRotation(0., 0., -1.);
	}

	void Gui::gtk_spnbTrlnStp_valueChanged()
	{
		sharer->setTranslationStep(spnbTrlnStp->get_value());
	}

	void Gui::gtk_spnbRttnStp_valueChanged()
	{
		sharer->setRotationStep(spnbRttnStp->get_value());
	}

	void Gui::gtk_restart_onButtonClick()
	{
		sharer->restartPose3D();
	}

	bool Gui::on_key_press_event(GdkEventKey* event)
	{
		switch ( event->keyval )
		{
			case 'W':
			case 'w':
				gtk_bttnTrlnFront_onButtonClick();
				break;
			case 'S':
			case 's':
				gtk_bttnTrlnBack_onButtonClick();
				break;
			case 'D':
			case 'd':
				gtk_bttnTrlnRight_onButtonClick();
				break;
			case 'A':
			case 'a':
				gtk_bttnTrlnLeft_onButtonClick();
				break;
			case 'R':
			case 'r':
				gtk_bttnTrlnUp_onButtonClick();
				break;
			case 'F':
			case 'f':
				gtk_bttnTrlnDown_onButtonClick();
				break;
			case 'E':
			case 'e':
				gtk_bttnRttnRight_onButtonClick();
				break;
			case 'Q':
			case 'q':
				gtk_bttnRttnLeft_onButtonClick();
				break;
			case 'T':
			case 't':
				gtk_bttnRttnUp_onButtonClick();
				break;
			case 'G':
			case 'g':
				gtk_bttnRttnDown_onButtonClick();
				break;
			case 'C':
			case 'c':
				gtk_bttnRttnCW_onButtonClick();
				break;
			case 'Z':
			case 'z':
				gtk_bttnRttnACW_onButtonClick();
				break;
		}

		return false;
	}


} /* namespace navigatorCamera */
