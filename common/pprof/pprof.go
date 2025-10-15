package pprof

import (
	"fmt"
	"k3c/common/logger"
	"net/http"
	"net/http/pprof"
	"strings"
)

func Run(addr string, prefix string) {
	var serveMux = &http.ServeMux{}
	if len(prefix) == 0 {
		prefix = "/debug"
	} else {
		prefix = "/" + strings.TrimLeft(prefix, "/")
	}
	serveMux.HandleFunc(prefix+"/", pprof.Index)
	serveMux.HandleFunc(prefix+"/cmdline", pprof.Cmdline)
	serveMux.HandleFunc(prefix+"/profile", pprof.Profile)
	serveMux.HandleFunc(prefix+"/symbol", pprof.Symbol)
	serveMux.HandleFunc(prefix+"/trace", pprof.Trace)
	serveMux.Handle(prefix+"/allocs", pprof.Handler("allocs"))
	serveMux.Handle(prefix+"/block", pprof.Handler("block"))
	serveMux.Handle(prefix+"/goroutine", pprof.Handler("goroutine"))
	serveMux.Handle(prefix+"/heap", pprof.Handler("heap"))
	serveMux.Handle(prefix+"/mutex", pprof.Handler("mutex"))
	serveMux.Handle(prefix+"/threadcreate", pprof.Handler("threadcreate"))

	if len(addr) == 0 {
		addr = ":6060"
	}
	srv := http.Server{
		Addr:    addr,
		Handler: serveMux,
	}
	go func() {
		if err := srv.ListenAndServe(); err != nil && err != http.ErrServerClosed {
			logger.Errorf("pprof listen %s error: %v", addr, err.Error())
		}
	}()

	fmt.Println("pprof ListenAndServe done")
}
