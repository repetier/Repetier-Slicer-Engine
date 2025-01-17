#include "MsgDialog.hpp"

#include <wx/settings.h>
#include <wx/sizer.h>
#include <wx/stattext.h>
#include <wx/button.h>
#include <wx/statbmp.h>
#include <wx/scrolwin.h>
#include <wx/clipbrd.h>
#include <wx/html/htmlwin.h>

#include <boost/algorithm/string/replace.hpp>

#include "libslic3r/libslic3r.h"
#include "libslic3r/Utils.hpp"
#include "GUI.hpp"
#include "I18N.hpp"
#include "ConfigWizard.hpp"
#include "wxExtensions.hpp"
#include "GUI_App.hpp"

namespace Slic3r {
namespace GUI {


MsgDialog::MsgDialog(wxWindow *parent, const wxString &title, const wxString &headline, wxWindowID button_id, wxBitmap bitmap)
	: wxDialog(parent, wxID_ANY, title, wxDefaultPosition, wxDefaultSize, wxDEFAULT_DIALOG_STYLE | wxRESIZE_BORDER)
	, boldfont(wxGetApp().normal_font()/*wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT)*/)
	, content_sizer(new wxBoxSizer(wxVERTICAL))
	, btn_sizer(new wxBoxSizer(wxHORIZONTAL))
{
	boldfont.SetWeight(wxFONTWEIGHT_BOLD);

    this->SetFont(wxGetApp().normal_font());

	auto *topsizer = new wxBoxSizer(wxHORIZONTAL);
	auto *rightsizer = new wxBoxSizer(wxVERTICAL);

	auto *headtext = new wxStaticText(this, wxID_ANY, headline);
	headtext->SetFont(boldfont);
    headtext->Wrap(CONTENT_WIDTH*wxGetApp().em_unit());
	rightsizer->Add(headtext);
	rightsizer->AddSpacer(VERT_SPACING);

	rightsizer->Add(content_sizer, 1, wxEXPAND);

	if (button_id != wxID_NONE) {
		auto *button = new wxButton(this, button_id);
		button->SetFocus();
		btn_sizer->Add(button);
	}

	rightsizer->Add(btn_sizer, 0, wxALIGN_RIGHT);

	if (! bitmap.IsOk()) {
		bitmap = create_scaled_bitmap("PrusaSlicer_192px.png", this, 192);
	}

	logo = new wxStaticBitmap(this, wxID_ANY, wxNullBitmap);

	topsizer->Add(logo, 0, /*wxALL*/wxTOP | wxBOTTOM | wxLEFT, BORDER);
	topsizer->Add(rightsizer, 1, wxALL | wxEXPAND, BORDER);

	SetSizerAndFit(topsizer);
}

// ErrorDialog

ErrorDialog::ErrorDialog(wxWindow *parent, const wxString &msg, bool monospaced_font)
    : MsgDialog(parent, wxString::Format(_(L("%s error")), SLIC3R_APP_NAME), 
                        wxString::Format(_(L("%s has encountered an error")), SLIC3R_APP_NAME),
		wxID_NONE)
	, msg(msg)
{
    // Text shown as HTML, so that mouse selection and Ctrl-V to copy will work.
    wxHtmlWindow* html = new wxHtmlWindow(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxHW_SCROLLBAR_AUTO);
    {
        html->SetMinSize(wxSize(40 * wxGetApp().em_unit(), monospaced_font ? 30 * wxGetApp().em_unit() : -1));
        wxFont 	  	font 			= wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT);
        wxFont      monospace       = wxGetApp().code_font();
		wxColour  	text_clr  		= wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOWTEXT);
        wxColour  	bgr_clr 		= wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOW);
		auto      	text_clr_str 	= wxString::Format(wxT("#%02X%02X%02X"), text_clr.Red(), text_clr.Green(), text_clr.Blue());
		auto      	bgr_clr_str 	= wxString::Format(wxT("#%02X%02X%02X"), bgr_clr.Red(), bgr_clr.Green(), bgr_clr.Blue());
		const int 	font_size       = font.GetPointSize()-1;
        int 		size[] 			= {font_size, font_size, font_size, font_size, font_size, font_size, font_size};
        html->SetFonts(font.GetFaceName(), monospace.GetFaceName(), size);
        html->SetBorders(2);
		std::string msg_escaped = xml_escape(msg.ToUTF8().data());
		boost::replace_all(msg_escaped, "\r\n", "<br>");
        boost::replace_all(msg_escaped, "\n", "<br>");
		if (monospaced_font)
			// Code formatting will be preserved. This is useful for reporting errors from the placeholder parser.
			msg_escaped = std::string("<pre><code>") + msg_escaped + "</code></pre>";
		html->SetPage("<html><body bgcolor=\"" + bgr_clr_str + "\"><font color=\"" + text_clr_str + "\">" + wxString::FromUTF8(msg_escaped.data()) + "</font></body></html>");
		content_sizer->Add(html, 1, wxEXPAND);
    }

	auto *btn_ok = new wxButton(this, wxID_OK);
	btn_ok->SetFocus();
	btn_sizer->Add(btn_ok, 0, wxRIGHT, HORIZ_SPACING);

	// Use a small bitmap with monospaced font, as the error text will not be wrapped.
	logo->SetBitmap(create_scaled_bitmap("PrusaSlicer_192px_grayscale.png", this, monospaced_font ? 48 : 192));

    SetMaxSize(wxSize(-1, CONTENT_MAX_HEIGHT*wxGetApp().em_unit()));
	Fit();
}


// InfoDialog

InfoDialog::InfoDialog(wxWindow* parent, const wxString &title, const wxString& msg)
	: MsgDialog(parent, wxString::Format(_L("%s information"), SLIC3R_APP_NAME), title)
	, msg(msg)
{
	this->SetBackgroundColour(wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOW));

	// Text shown as HTML, so that mouse selection and Ctrl-V to copy will work.
	wxHtmlWindow* html = new wxHtmlWindow(this, wxID_ANY, wxDefaultPosition, wxDefaultSize, wxHW_SCROLLBAR_AUTO);
	{
		wxFont 	  	font = wxSystemSettings::GetFont(wxSYS_DEFAULT_GUI_FONT);
		wxFont      monospace = wxGetApp().code_font();
		wxColour  	text_clr = wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOWTEXT);
		wxColour  	bgr_clr = wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOW);
		auto      	text_clr_str = wxString::Format(wxT("#%02X%02X%02X"), text_clr.Red(), text_clr.Green(), text_clr.Blue());
		auto      	bgr_clr_str = wxString::Format(wxT("#%02X%02X%02X"), bgr_clr.Red(), bgr_clr.Green(), bgr_clr.Blue());
		const int 	font_size = font.GetPointSize() - 1;
		int 		size[] = { font_size, font_size, font_size, font_size, font_size, font_size, font_size };
		html->SetFonts(font.GetFaceName(), monospace.GetFaceName(), size);
		html->SetBorders(2);

		// calculate html page size from text
		int lines = msg.Freq('\n');

		if (msg.Contains("<tr>")) {
			int pos = 0;
			while (pos < (int)msg.Len() && pos != wxNOT_FOUND) {
				pos = msg.find("<tr>", pos + 1);
				lines+=2;
			}
		}
		int page_height = std::min((font.GetPixelSize().y + 1) * lines, 68 * wxGetApp().em_unit());
		wxSize page_size(68 * wxGetApp().em_unit(), page_height);

		html->SetMinSize(page_size);

		std::string msg_escaped = xml_escape(msg.ToUTF8().data(), true);
		boost::replace_all(msg_escaped, "\r\n", "<br>");
		boost::replace_all(msg_escaped, "\n", "<br>");
		html->SetPage("<html><body bgcolor=\"" + bgr_clr_str + "\"><font color=\"" + text_clr_str + "\">" + wxString::FromUTF8(msg_escaped.data()) + "</font></body></html>");
		content_sizer->Add(html, 1, wxEXPAND);
	}

	// Set info bitmap
	logo->SetBitmap(create_scaled_bitmap("info", this, 84));

	Fit();
}


}
}
