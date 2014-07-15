
/**
Copyright 2014 Joseph Lewis III <joseph@josephlewis.net> All rights reserved.
Use of this source code is governed by a BSD license.

The autopilot_web program creates a server at port 9000 that listens for HTTP
requests and will run configurations with autopilots in any given selection and
also allows the downloading of log files
**/

package main

import (
	"html/template"
	"net/http"
	"io/ioutil"
	"fmt"
	"path/filepath"
	"os"
	"os/exec"
	"log"
	"io"
	"archive/zip"
	"bytes"
	"path"
	"time"
)

var templates = template.Must(template.ParseFiles("templates/root.html", "templates/command.html"))
var started_proc *exec.Cmd = nil

func rootHandler(w http.ResponseWriter, r *http.Request) {
	GenerateMainPage(w, "")
}

func ZipFolder(path string) (zippedFolder []byte, err error){
	var buf bytes.Buffer
	w := zip.NewWriter(&buf)
	wr, err := w.Create("archive_info.txt")
	if err != nil {
		return nil, err
	}

	t := time.Now().Local()
	wr.Write([]byte("Created by the autopilot webserver on " +  t.Format("02 Jan 2006 15:04:05 MST")))


	err = filepath.Walk(path, func(path string, f os.FileInfo, err error) error {

		if f.IsDir() {
			return nil
		}

		wr, err = w.Create(f.Name())
		if(err != nil) {
			return err
		}

		fileBytes, err := ioutil.ReadFile(path)
		if(err != nil) {
			return err
		}
		_, err = wr.Write(fileBytes)
		return err
	})

	if err != nil {
		return nil, err
	}

	w.Close()
	return buf.Bytes(), nil
}

/// Stops the autopilot if it is running
func StopAutopilot(w http.ResponseWriter, r * http.Request) {

	// Check to make sure an autopilot isn't started yet.
	if AutopilotRunning() {
		msg := "Autopilot stopped"
		err := started_proc.Process.Kill()
		if(err != nil) {
			msg = err.Error()
		}
		time.Sleep(500)
		GenerateMainPage(w, msg)
	} else {
		GenerateMainPage(w, "Autopilot not running")
	}
}

func StartAutopilot(w http.ResponseWriter, r * http.Request) {

	// Check to make sure an autopilot isn't started yet.
	if AutopilotRunning() {
		GenerateMainPage(w, "Autopilot already running, can't start another!")
		return

	}

	// Make the directory for the autopilot run
	t := time.Now().Local()
	run, _ := filepath.Abs("./autopilot/runs/" + t.Format("02 Jan 2006 15:04:05 MST"))
	mode := os.ModeDir | 0777;
	err := os.MkdirAll(run, mode)
	if err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}

	// Copy the autopilot executable and configuration
	pilot := r.FormValue("autopilot")
	config := r.FormValue("configuration")

	err = copyFileContents("./autopilot/pilots/" + pilot, run + "/autopilot")
	if err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}

	_ = os.Chmod(run + "/autopilot", 0777)


	err = copyFileContents("./autopilot/configurations/" + config, run + "/config.xml")
	if err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}

	// Start the autopilot
	started_proc = exec.Command(run + "/autopilot")
	started_proc.Dir = run
	err = started_proc.Start()
	if err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}


	// Start a watcher on the started process so we can clean up when it dies
	go func() {
		started_proc.Wait()
		started_proc = nil
		log.Print("Autopilot finished")
	}()  // Note the parentheses - must call the function.


	GenerateMainPage(w, "Started Autopilot")


}

func AutopilotDownloader(w http.ResponseWriter, r * http.Request) {
	log.Print(r.URL.Path)
	_, file := path.Split(r.URL.Path)
	bytes, err := ZipFolder("./autopilot/runs/" + file)

	if (err != nil) {
		http.Error(w, err.Error(), http.StatusInternalServerError)
		return
	}

	w.Header().Add("Content-Type", "application/zip")
	w.Header().Add("Content-Length", fmt.Sprintf("%d", len(bytes)) )
	w.Write(bytes)
}

func ListDirectory(dir string) []string {
	var runs []string

	files, _ := ioutil.ReadDir(dir)
	for _, f := range files {
		runs = append(runs, f.Name())
	}

	return runs;
}

func AutopilotRunning() bool {
	if started_proc == nil {
		return false
	}

	if started_proc.ProcessState != nil {
		return false
	}

	return true
}

func GenerateMainPage(w http.ResponseWriter, notification string) {
	pageData := make(map[string] interface{})

	pageData["alert"] = notification

	pageData["runs"] = ListDirectory("./autopilot/runs")
	pageData["pilots"] = ListDirectory("./autopilot/pilots")
	pageData["configurations"] = ListDirectory("./autopilot/configurations")

	running := AutopilotRunning()
	pageData["proc"] = running


	err := templates.ExecuteTemplate(w, "root.html", pageData)
	if err != nil {
		http.Error(w, err.Error(), http.StatusInternalServerError)
	}

}

func GenericUploader(basepath string) func(http.ResponseWriter, *http.Request) {
	return func (w http.ResponseWriter, r *http.Request) {
		f, _, err := r.FormFile("path")
		n := r.FormValue("name")

		if err != nil {
			GenerateMainPage(w, err.Error())
			return
		}

		t := time.Now().Local()


		fd, err := os.Create(basepath + n + " " + t.Format("2006-01-02 15:04:05"))
		if err != nil {
			GenerateMainPage(w, err.Error())
			return
		}
		defer fd.Close()

		// Write file field from file to upload
		_, err = io.Copy(fd, f)
		if err != nil {
			GenerateMainPage(w, err.Error())
			return
		}

		GenerateMainPage(w, "Uploaded!")
	}
}


func GenericDeleter(basepath string) func(http.ResponseWriter, *http.Request) {
	return func (w http.ResponseWriter, r *http.Request) {
		_, file := path.Split(r.URL.Path)
		err := os.RemoveAll(basepath + file)

		res := "Deleted Successfully!"
		if (err != nil) {
			res = err.Error()
		}

		GenerateMainPage(w, res)
	}
}

func GenerateFolders() (error) {
	// Make all the requisite directories for holding things.
	autopilotpath, _ := filepath.Abs("./autopilot/")
	configurations, _ := filepath.Abs("./autopilot/configurations")
	current, _ := filepath.Abs("./autopilot/configurations")
	autopilots, _ := filepath.Abs("./autopilot/pilots")
	runs, _ := filepath.Abs("./autopilot/runs")

	mode := os.ModeDir | 0777;

	err := os.MkdirAll(autopilotpath, mode)
	if (err != nil) {
		return err
	}

	err = os.MkdirAll(configurations, mode)
	if (err != nil) {
		return err
	}

	err = os.MkdirAll(current, mode)
	if (err != nil) {
		return err
	}

	err = os.MkdirAll(autopilots, mode)
	if (err != nil) {
		return err
	}
	err = os.MkdirAll(runs, mode)
	if (err != nil) {
		return err
	}
	return nil;
}


// CopyFile copies a file from src to dst. If src and dst files exist, and are
// the same, then return success. Otherise, attempt to create a hard link
// between the two files. If that fail, copy the file contents from src to dst.
func CopyFile(src, dst string) (err error) {
    sfi, err := os.Stat(src)
    if err != nil {
        return
    }
    if !sfi.Mode().IsRegular() {
        // cannot copy non-regular files (e.g., directories,
        // symlinks, devices, etc.)
        return fmt.Errorf("CopyFile: non-regular source file %s (%q)", sfi.Name(), sfi.Mode().String())
    }
    dfi, err := os.Stat(dst)
    if err != nil {
        if !os.IsNotExist(err) {
            return
        }
    } else {
        if !(dfi.Mode().IsRegular()) {
            return fmt.Errorf("CopyFile: non-regular destination file %s (%q)", dfi.Name(), dfi.Mode().String())
        }
        if os.SameFile(sfi, dfi) {
            return
        }
    }
    if err = os.Link(src, dst); err == nil {
        return
    }
    err = copyFileContents(src, dst)
    return
}

// http://stackoverflow.com/a/21067803
// copyFileContents copies the contents of the file named src to the file named
// by dst. The file will be created if it does not already exist. If the
// destination file exists, all it's contents will be replaced by the contents
// of the source file.
func copyFileContents(src, dst string) (err error) {
    in, err := os.Open(src)
    if err != nil {
        return
    }
    defer in.Close()
    out, err := os.Create(dst)
    if err != nil {
        return
    }
    defer func() {
        cerr := out.Close()
        if err == nil {
            err = cerr
        }
    }()
    if _, err = io.Copy(out, in); err != nil {
        return
    }
    err = out.Sync()
    return
}


func GenericCommand(command string, args ...string) func(http.ResponseWriter, *http.Request) {
	return func (w http.ResponseWriter, r *http.Request) {

		//go func(){
			sp := exec.Command(command, args...)
			bytes, err := sp.CombinedOutput()
			//GenerateMainPage(w, "Running " + command)
		//}()

		err = templates.ExecuteTemplate(w, "command.html", string(bytes))
		if err != nil {
			http.Error(w, err.Error(), http.StatusInternalServerError)
		}

	}
}


func main() {

	err := GenerateFolders()
	fmt.Println(err)


	includepath, _ := filepath.Abs("./static/")

	http.Handle("/inc/", http.StripPrefix("/inc/", http.FileServer(http.Dir(includepath))))
	http.HandleFunc("/log/download/", AutopilotDownloader)
	http.HandleFunc("/log/delete/", GenericDeleter("./autopilot/runs/"))
	http.HandleFunc("/autopilot/upload/", GenericUploader("./autopilot/pilots/"))
	http.HandleFunc("/autopilot/delete/", GenericDeleter("./autopilot/pilots/"))
	http.HandleFunc("/autopilot/start/", StartAutopilot)
	http.HandleFunc("/autopilot/stop/", StopAutopilot)
	http.HandleFunc("/configuration/upload/", GenericUploader("./autopilot/configurations/"))
	http.HandleFunc("/configuration/delete/", GenericDeleter("./autopilot/configurations/"))
	http.HandleFunc("/command/shutdown/", GenericCommand("/sbin/shutdown", "-H", "-P", "now"))
	http.HandleFunc("/command/proc/", GenericCommand("/bin/ps", "-aux"))
	http.HandleFunc("/command/ifconfig/", GenericCommand("/sbin/ifconfig"))
	http.HandleFunc("/command/dmesg/", GenericCommand("/bin/dmesg"))
	http.HandleFunc("/command/w/", GenericCommand("/usr/bin/w"))
	http.HandleFunc("/command/cpuinfo/", GenericCommand("/bin/cat", "/proc/cpuinfo"))



	http.Handle("/configuration/download/", http.StripPrefix("/configuration/download/", http.FileServer(http.Dir("./autopilot/configurations/"))))
	http.Handle("/autopilot/download/", http.StripPrefix("/autopilot/download/", http.FileServer(http.Dir("./autopilot/pilots/"))))

	http.HandleFunc("/", rootHandler)

	fmt.Println("Running")
	err = http.ListenAndServe("localhost:9000", nil)
	fmt.Println(err)
}
