
epoll_reactor::epoll_reactor()
  : epoll_fd_(do_epoll_create()) {
    ...
    epoll_ctl(epoll_fd_, EPOLL_CTL_ADD, interrupter_.read_descriptor(), &ev);
}

void epoll_reactor::run(...) {
    ...
    epoll_wait(epoll_fd_, events, 128, timeout);
}

void epoll_reactor::interrupt() {
    ...
    epoll_ctl(epoll_fd_, EPOLL_CTL_MOD, interrupter_.read_descriptor(), &ev);
}

int epoll_reactor::do_epoll_create() {
    ...
    fd = epoll_create(epoll_size);
    return fd;
}
